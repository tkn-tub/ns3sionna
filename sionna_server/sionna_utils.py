import numpy as np

c = 299792458

def compute_coherence_time(v, fc, model='rappaport2'):
    """
    Computes the channel coherence time
    :param v: mobile speed in m/s
    :param fc: center frequency in Hz
    :param model: model for computation
    :return: Tc in ns
    """
    assert 0 <= v <= 100
    assert fc >= 1e6 # > 1MHz

    if model == 'rappaport':
        Tc = int(np.ceil(9 * c * 1e9 / (16 * np.pi * 2 * v * fc)))
    elif model == 'rappaport2':
        Tc = int(np.ceil(0.423 * c * 1e9 / (v * fc)))
    return Tc


if __name__ == '__main__':
    v = 1.0 # m/s
    fc = 5210e6 # center freq

    Tc1 = compute_coherence_time(v, fc, model='rappaport')
    print(f'{Tc1}ns -> {Tc1/1e6}ms')

    Tc2 = compute_coherence_time(v, fc, model='rappaport2')
    print(f'{Tc2}ns -> {Tc2/1e6}ms')