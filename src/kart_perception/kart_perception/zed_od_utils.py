"""Utility to convert ZED ObjectsStamped → Detection3DArray.

Used by cone_follower, steering_hud, and cone_marker_viz_3d to natively
subscribe to ZED SDK built-in object detection without a bridge node.
"""
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

try:
    from zed_interfaces.msg import ObjectsStamped  # noqa: F401
    HAS_ZED_INTERFACES = True
except ImportError:
    HAS_ZED_INTERFACES = False

# Map numeric class IDs (from ONNX model metadata) to canonical names.
# ZED SDK may publish the raw class index as the label string.
_CLASS_ID_MAP = {
    "0": "blue_cone",
    "1": "yellow_cone",
    "2": "orange_cone",
    "3": "large_orange_cone",
}


def zed_objects_to_det3d(msg) -> Detection3DArray:
    """Convert a ZED ObjectsStamped message to a Detection3DArray."""
    out = Detection3DArray()
    out.header = msg.header

    for obj in msg.objects:
        label = _CLASS_ID_MAP.get(obj.label, obj.label)
        score = obj.confidence / 100.0
        x = float(obj.position[0])
        y = float(obj.position[1])
        z = float(obj.position[2])

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = label
        hyp.hypothesis.score = score
        hyp.pose.pose.position.x = x
        hyp.pose.pose.position.y = y
        hyp.pose.pose.position.z = z
        hyp.pose.pose.orientation.w = 1.0

        bbox = BoundingBox3D()
        bbox.center.position.x = x
        bbox.center.position.y = y
        bbox.center.position.z = z
        bbox.center.orientation.w = 1.0
        bbox.size.x = bbox.size.y = bbox.size.z = 0.25

        det = Detection3D()
        det.header = msg.header
        det.results.append(hyp)
        det.bbox = bbox
        out.detections.append(det)

    return out
