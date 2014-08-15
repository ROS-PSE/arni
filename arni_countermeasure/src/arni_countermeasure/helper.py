import rospy

#: the parameter namespace for the arni_countermeasure node
ARNI_CTM_NS = "/arni/countermeasure/"

#: the parameter namespace for configuration files of the arni_countermeasure node
ARNI_CTM_CFG_NS = ARNI_CTM_NS + "config/"

def get_param_duration(param):
    """Calls rospy.get_param and logs errors.

    Logs if the param does not exist or is not parsable to rospy.Durotation.
    And calls rospy.signal_shutdown if the value is invalid/not existing.

    :return:    The Param param from the parameter server.
    :rtype: rospy.Duration

    """

    # dummy value
    value = rospy.Duration(1)

    try:
        # only a default value in case the param gets fuzzed.
        value = rospy.Duration(
            rospy.get_param(param))
    except KeyError:
        err_msg = (
            "Param %s is not set" % param
            + "and its default value has been forcefully removed")
        rospy.logerr(err_msg)
        rospy.signal_shutdown(err_msg)
    except ValueError:
        err_msg = (
            "Param %s has the invalid value '%s'."
            % (param, rospy.get_param(param)))
        rospy.logerr(err_msg)
        rospy.signal_shutdown(err_msg)
    return value
