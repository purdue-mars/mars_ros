import rospy
from typing import Dict, Any, List
import torch.nn.functional as F
import numpy as np
import cv2
import torch
from mars_perception.object_face_classifier.model import SimpleCNN, SimpleClassifierNN
from mars_perception.object_face_classifier.util import obj_center_crop

class EvalObjectFaceClassifier:
    """
    Uses trained NN to classify object faces
    execute() -> int (class)
    Params:
      - model_path (Required)
      - compute_type (default: 'cpu')
      - axes (Required)
    """

    # Parameter defaults
    _compute_type: str = "cuda"

    def __init__(self, cfg: Dict[str, Any]):
        super().__init__()

        if "model_path" not in cfg:
            raise RuntimeError("Missing model_path.")
        if "compute_type" in cfg:
            self._compute_type: str = cfg["compute_type"]
        else:
            raise RuntimeError("Missing compute_type.")
        if "axes" in cfg:
            self._axes: List[List[int]] = cfg["axes"]
        else:
            raise RuntimeError("Missing axes.")
        if "input_dim" in cfg:
            self._input_dim: List[str] = cfg["input_dim"]
        else:
            raise RuntimeError("Missing input_shape.")
        if "min_features" in cfg:
            self._min_features: int = cfg["min_features"]
        else:
            raise RuntimeError("Missing min_features.")

        self._device = torch.device(self._compute_type)
        self._model = SimpleCNN(self._input_dim,len(self._axes))
        self._model.to(self._device)
        self._model.load_state_dict(torch.load(cfg["model_path"]))
        self._model.eval()

    def run(self, frame: cv2.Mat):
        feats = len(np.argwhere(frame != 0))
        if(feats > self._min_features):
            frame = obj_center_crop(frame,self._input_dim)
            assert frame.shape == tuple(self._input_dim)
            frame: torch.Tensor = torch.from_numpy(np.float32(frame)).to(self._device)
            frame = torch.unsqueeze(frame,0)
            out: torch.Tensor = self._model(frame)
            out = out.detach().numpy().argmax()
            return out 
        else:
            return -1