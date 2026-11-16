import math

from blueboat_ppo_controller.thrust_calibration import (
    StreamCheck,
    post_command_feedback_ready,
)


def test_stable_two_hz_output_is_accepted():
    check = StreamCheck(max_age_s=.65, max_gap_s=.65, window_s=2.)
    for receipt in (0., .5, 1., 1.5, 2.):
        check.accept(100.+receipt, 100.+receipt, receipt)

    report = check.report(2., 102.)
    assert report['fresh'] is True
    assert math.isclose(report['rate_hz'], 2.)
    assert math.isclose(report['max_gap_s'], .5)


def test_one_second_output_gap_is_rejected():
    check = StreamCheck(max_age_s=.65, max_gap_s=.65, window_s=2.)
    for receipt in (0., .5, 1.5, 2.):
        check.accept(100.+receipt, 100.+receipt, receipt)

    report = check.report(2., 102.)
    assert report['max_gap_s'] == 1.
    assert report['max_gap_s'] > check.max_gap


def test_response_requires_feedback_newer_than_command():
    assert not post_command_feedback_ready(10., 9.9, 10.7, .65)
    assert not post_command_feedback_ready(10., 10.5, 10.6, .65)
    assert post_command_feedback_ready(10., 10.5, 10.7, .65)


def test_repeated_publish_does_not_make_stale_feedback_look_new():
    command_time = 10.
    output_receipt = 10.5
    assert post_command_feedback_ready(command_time, output_receipt, 10.7, .65)
    assert not post_command_feedback_ready(10.6, output_receipt, 11.3, .65)
