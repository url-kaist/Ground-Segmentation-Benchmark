import numpy as np

def intensity2labels(intensity_np):
    label = intensity_np.astype(np.uint32)
    sem_label = label & 0xFFFF  # semantic label in lower half
    inst_label = label >> 16  # instance id in upper half
    return sem_label, inst_label
