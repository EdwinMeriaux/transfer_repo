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


def test_expected_one_point_one_second_output_gap_is_accepted():
    check = StreamCheck(max_age_s=1.25, max_gap_s=1.2, window_s=3.)
    for receipt in (0., 1.1, 2.2):
        check.accept(100.+receipt, 100.+receipt, receipt)

    report = check.report(2.2, 102.2)
    assert math.isclose(report['rate_hz'], 1/1.1)
    assert math.isclose(report['max_gap_s'], 1.1)
    assert report['max_gap_s'] <= check.max_gap

    between_samples = check.report(3.19, 103.19)
    assert math.isclose(between_samples['rate_hz'], 1/1.1)


def test_two_second_output_gap_is_rejected():
    check = StreamCheck(max_age_s=1.25, max_gap_s=1.2, window_s=3.)
    for receipt in (0., 2.):
        check.accept(100.+receipt, 100.+receipt, receipt)

    report = check.report(2., 102.)
    assert report['max_gap_s'] == 2.
    assert report['max_gap_s'] > check.max_gap


def test_response_requires_feedback_newer_than_command():
    assert not post_command_feedback_ready(10., 9.9, 11.3, 1.25)
    assert not post_command_feedback_ready(10., 11.1, 11.2, 1.25)
    assert post_command_feedback_ready(10., 11.1, 11.3, 1.25)


def test_repeated_publish_does_not_make_stale_feedback_look_new():
    command_time = 10.
    output_receipt = 11.1
    assert post_command_feedback_ready(command_time, output_receipt, 11.3, 1.25)
    assert not post_command_feedback_ready(11.2, output_receipt, 12.5, 1.25)
