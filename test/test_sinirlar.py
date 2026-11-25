import numpy as np

from oda_kesif import sinirlar


def _oda():
    """20x20 harita: sol yarı keşfedilmiş (boş), sağ yarı bilinmiyor, kenarlar duvar."""
    g = np.full((20, 20), -1)
    g[:, :10] = 0
    g[0, :10] = 100
    g[-1, :10] = 100
    g[:, 0] = 100
    return g


def test_komsu_var_halka():
    m = np.zeros((5, 5), dtype=bool)
    m[2, 2] = True
    sonuc = sinirlar.komsu_var(m)
    assert sonuc.sum() == 8          # 8 komşu
    assert not sonuc[2, 2]           # hücrenin kendisi değil


def test_sinir_hucreleri_sinir_cizgisinde():
    m = sinirlar.sinir_hucreleri(_oda())
    assert m[:, 9].sum() > 0          # sınır sütununda frontier var
    assert m[:, :9].sum() == 0        # içeride frontier yok


def test_kumele_capraz_komsular_tek_kume():
    m = np.zeros((6, 8), dtype=bool)
    m[1:4, 1] = True
    m[4, 6] = True
    m[5, 7] = True
    assert len(sinirlar.kumele(m)) == 2


def test_tek_kume_ve_dogru_hedef():
    s = sinirlar.sinirlari_bul(_oda(), 0.05, engel_payi_m=0.1, min_boyut_m=0.3)
    assert len(s) == 1
    (satir, sutun), boyut = s[0]
    assert (satir, sutun) == (9, 9)
    assert abs(boyut - 0.70) < 1e-9


def test_tamamen_bilinen_haritada_frontier_yok():
    g = np.zeros((10, 10), dtype=int)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    assert sinirlar.sinirlari_bul(g, 0.05) == []


def test_kucuk_kumeler_elenir():
    g = np.zeros((30, 30), dtype=int)
    g[15, 15] = -1                    # tek bilinmeyen hücre → minik frontier
    assert sinirlar.sinirlari_bul(g, 0.05, min_boyut_m=0.5) == []