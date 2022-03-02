import numpy as np


from tqdm import tqdm
a= np.array([1, 2])
b= np.array([1, 43])
for q, w, z in tqdm(zip(a, b, b)):
    print(q, w, z)