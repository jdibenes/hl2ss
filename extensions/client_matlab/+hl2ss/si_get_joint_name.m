
function [name] = si_get_joint_name(joint_kind)
if (joint_kind == si_hand_joint_kind.Palm)
    name = "Palm";
elseif (joint_kind == si_hand_joint_kind.Wrist)
    name = "Wrist";
elseif (joint_kind == si_hand_joint_kind.ThumbMetacarpal)
    name = "ThumbMetacarpal";
elseif (joint_kind == si_hand_joint_kind.ThumbProximal)
    name = "ThumbProximal";
elseif (joint_kind == si_hand_joint_kind.ThumbDistal)
    name = "ThumbDistal";
elseif (joint_kind == si_hand_joint_kind.ThumbTip)
    name = "ThumbTip";
elseif (joint_kind == si_hand_joint_kind.IndexMetacarpal)
    name = "IndexMetacarpal";
elseif (joint_kind == si_hand_joint_kind.IndexProximal)
    name = "IndexProximal";
elseif (joint_kind == si_hand_joint_kind.IndexIntermediate)
    name = "IndexIntermediate";
elseif (joint_kind == si_hand_joint_kind.IndexDistal)
    name = "IndexDistal";
elseif (joint_kind == si_hand_joint_kind.IndexTip)
    name = "IndexTip";
elseif (joint_kind == si_hand_joint_kind.MiddleMetacarpal)
    name = "MiddleMetacarpal";
elseif (joint_kind == si_hand_joint_kind.MiddleProximal)
    name = "MiddleProximal";
elseif (joint_kind == si_hand_joint_kind.MiddleIntermediate)
    name = "MiddleIntermediate";
elseif (joint_kind == si_hand_joint_kind.MiddleDistal)
    name = "MiddleDistal";
elseif (joint_kind == si_hand_joint_kind.MiddleTip)
    name = "MiddleTip";
elseif (joint_kind == si_hand_joint_kind.RingMetacarpal)
    name = "RingMetacarpal";
elseif (joint_kind == si_hand_joint_kind.RingProximal)
    name = "RingProximal";
elseif (joint_kind == si_hand_joint_kind.RingIntermediate)
    name = "RingIntermediate";
elseif (joint_kind == si_hand_joint_kind.RingDistal)
    name = "RingDistal";
elseif (joint_kind == si_hand_joint_kind.RingTip)
    name = "RingTip";
elseif (joint_kind == si_hand_joint_kind.LittleMetacarpal)
    name = "LittleMetacarpal";
elseif (joint_kind == si_hand_joint_kind.LittleProximal)
    name = "LittleProximal";
elseif (joint_kind == si_hand_joint_kind.LittleIntermediate)
    name = "LittleIntermediate";
elseif (joint_kind == si_hand_joint_kind.LittleDistal)
    name = "LittleDistal";
elseif (joint_kind == si_hand_joint_kind.LittleTip)
    name = "LittleTip";
else
    name = "";
end
end
