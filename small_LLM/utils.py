import numpy as np


def get_rand_scale_vec(vec_len):
    idx = np.random.randint(1, 2**vec_len)
    scales = []
    for i in range(vec_len-1, -1, -1):
        scales.append(idx // 2**i)
        idx %= 2**i

    return scales
