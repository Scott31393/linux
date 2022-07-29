// SPDX-License-Identifier: GPL-2.0-only
/*
 * vivid-kthread-touch.c - touch capture thread support functions.
 *
 */

#include <linux/freezer.h>
#include <linux/jiffies.h>
#include <asm/div64.h>
#include "vivid-core.h"
#include "vivid-kthread-touch.h"
#include "vivid-touch-cap.h"

static noinline_for_stack void vivid_thread_tch_cap_tick(struct vivid_dev *dev,
														 u64 ts,
														 int dropped_bufs)
{
	struct vivid_buffer *tch_cap_buf = NULL;

	spin_lock(&dev->slock);
	if (!list_empty(&dev->touch_cap_active)) {
		tch_cap_buf = list_entry(dev->touch_cap_active.next,
					 struct vivid_buffer, list);
		list_del(&tch_cap_buf->list);
	}

	spin_unlock(&dev->slock);

	if (tch_cap_buf) {
		v4l2_ctrl_request_setup(tch_cap_buf->vb.vb2_buf.req_obj.req,
					&dev->ctrl_hdl_touch_cap);

		vivid_fillbuff_tch(dev, tch_cap_buf);
		v4l2_ctrl_request_complete(tch_cap_buf->vb.vb2_buf.req_obj.req,
					   &dev->ctrl_hdl_touch_cap);
		vb2_buffer_done(&tch_cap_buf->vb.vb2_buf, dev->dqbuf_error ?
				VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
		dprintk(dev, 2, "touch_cap buffer %d done\n",
			tch_cap_buf->vb.vb2_buf.index);

		tch_cap_buf->vb.vb2_buf.timestamp = ts + dev->time_wrap_offset;
	}
	dev->dqbuf_error = false;
}

static void vivid_touch_cap_work(struct work_struct *work)
{
	struct vivid_dev *dev = container_of(work,
							struct vivid_dev, work_touch_cap);
	u64 numerators_since_start;
	u64 buffers_since_start;
	u64 next_jiffies_since_start;
	unsigned long jiffies_since_start;
	unsigned long cur_jiffies;
	unsigned int wait_jiffies;
	unsigned int numerator;
	unsigned int denominator;
	int dropped_bufs;

	dprintk(dev, 1, "Touch Capture Thread Start\n");

	set_freezable();

	/* Resets frame counters */
	dev->touch_cap_seq_offset = 0;
	dev->touch_cap_seq_count = 0;
	dev->touch_cap_seq_resync = false;
	dev->jiffies_touch_cap = jiffies;
	if (dev->time_wrap)
		dev->time_wrap_offset = dev->time_wrap - ktime_get_ns();
	else
		dev->time_wrap_offset = 0;

	for (;;) {
		try_to_freeze();
		if (kthread_should_stop())
			break;

		if (!mutex_trylock(&dev->mutex)) {
			schedule();
			continue;
		}
		cur_jiffies = jiffies;
		if (dev->touch_cap_seq_resync) {
			dev->jiffies_touch_cap = cur_jiffies;
			dev->touch_cap_seq_offset = dev->touch_cap_seq_count + 1;
			dev->touch_cap_seq_count = 0;
			dev->cap_seq_resync = false;
		}
		denominator = dev->timeperframe_tch_cap.denominator;
		numerator = dev->timeperframe_tch_cap.numerator;

		/* Calculate the number of jiffies since we started streaming */
		jiffies_since_start = cur_jiffies - dev->jiffies_touch_cap;
		/* Get the number of buffers streamed since the start */
		buffers_since_start = (u64)jiffies_since_start * denominator +
				      (HZ * numerator) / 2;
		do_div(buffers_since_start, HZ * numerator);

		/*
		 * After more than 0xf0000000 (rounded down to a multiple of
		 * 'jiffies-per-day' to ease jiffies_to_msecs calculation)
		 * jiffies have passed since we started streaming reset the
		 * counters and keep track of the sequence offset.
		 */
		if (jiffies_since_start > JIFFIES_RESYNC) {
			dev->jiffies_touch_cap = cur_jiffies;
			dev->cap_seq_offset = buffers_since_start;
			buffers_since_start = 0;
		}
		dropped_bufs = buffers_since_start + dev->touch_cap_seq_offset - dev->touch_cap_seq_count;
		dev->touch_cap_seq_count = buffers_since_start + dev->touch_cap_seq_offset;
		dev->touch_cap_with_seq_wrap_count =
			dev->touch_cap_seq_count - dev->touch_cap_seq_start;

		vivid_thread_tch_cap_tick(dev, ktime_get_ns(), dropped_bufs);

		/*
		 * Calculate the number of 'numerators' streamed
		 * since we started, including the current buffer.
		 */
		numerators_since_start = ++buffers_since_start * numerator;

		/* And the number of jiffies since we started */
		jiffies_since_start = jiffies - dev->jiffies_touch_cap;

		mutex_unlock(&dev->mutex);

		/*
		 * Calculate when that next buffer is supposed to start
		 * in jiffies since we started streaming.
		 */
		next_jiffies_since_start = numerators_since_start * HZ +
					   denominator / 2;
		do_div(next_jiffies_since_start, denominator);
		/* If it is in the past, then just schedule asap */
		if (next_jiffies_since_start < jiffies_since_start)
			next_jiffies_since_start = jiffies_since_start;

		wait_jiffies = next_jiffies_since_start - jiffies_since_start;
		while (time_is_after_jiffies(cur_jiffies + wait_jiffies) &&
		       !kthread_should_stop())
			schedule();
	}
	dprintk(dev, 1, "Touch Capture Thread End\n");
}

static enum hrtimer_restart vivid_hrtimer_touch_cap(struct hrtimer *hrtimer)
{
	unsigned int wait_send_next_ns = 1000;
	struct vivid_dev *dev = container_of(hrtimer,
							struct vivid_dev, hrtimer_touch_cap);

	if (!work_busy(&dev->work_touch_cap))
		schedule_work(&dev->work_touch_cap);

	hrtimer_forward_now(hrtimer, ns_to_ktime(wait_send_next_ns));
	return HRTIMER_RESTART;
}


int vivid_start_generating_touch_cap(struct vivid_dev *dev)
{
	unsigned int init_hrtimer_tick_ns;
	struct hrtimer *hrt = &dev->hrtimer_touch_cap;
	init_hrtimer_tick_ns = 1000000000;

	/* TODO: Check this */
	if (dev->touch_cap_streaming) {
		dprintk(dev, 1, "already streaming returning from %s\n", __func__);
		return 0;
	}

	dev->touch_cap_seq_start = dev->seq_wrap * 128;
	hrtimer_init(hrt, CLOCK_REALTIME, HRTIMER_MODE_ABS);
	hrt->function = vivid_hrtimer_touch_cap;
	hrtimer_start(hrt, ns_to_ktime(init_hrtimer_tick_ns),
				  HRTIMER_MODE_ABS);

	INIT_WORK(&dev->work_touch_cap, vivid_touch_cap_work);

	dev->touch_cap_streaming = true;
	dprintk(dev, 1, "returning from %s\n", __func__);
	return 0;
}

void vivid_stop_generating_touch_cap(struct vivid_dev *dev)
{
	struct hrtimer *hrt = &dev->hrtimer_touch_cap;

	/* TODO: Check this */
	if (!hrt)
		return;

	dev->touch_cap_streaming = false;

	while (!list_empty(&dev->touch_cap_active)) {
		struct vivid_buffer *buf;

		buf = list_entry(dev->touch_cap_active.next,
				 struct vivid_buffer, list);
		list_del(&buf->list);
		v4l2_ctrl_request_complete(buf->vb.vb2_buf.req_obj.req,
					   &dev->ctrl_hdl_touch_cap);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		dprintk(dev, 2, "touch_cap buffer %d done\n",
			buf->vb.vb2_buf.index);
	}

	hrtimer_cancel(hrt);
}
