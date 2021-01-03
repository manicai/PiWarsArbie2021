import arbie.constants as constants


def test_is_cross():
    for k in [constants.PadKeys.cross_up, constants.PadKeys.cross_down]:
        assert k.is_cross()

    for k in [constants.PadKeys.right_shoulder, constants.PadKeys.button_x]:
        assert not k.is_cross()
