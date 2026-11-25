"""Frontier bulma: saf NumPy, ROS'tan bağımsız."""
from collections import deque

import numpy as np

BILINMIYOR = -1
BOS_ESIK = 25
DOLU_ESIK = 65

def komsu_var(maske):
    """8 komşusundan en az biri True olan hücreler."""
    h, w = maske.shape
    p = np.pad(maske,1,constant_values=False)
    sonuc = np.zeros_like(maske)
    for dy in (-1,0,1):
        for dx in (-1,0,1):
            if (dy,dx) != (0,0):
                sonuc |= p[1 + dy:1 + dy + h, 1 + dx:1 + dx + w]
    return sonuc

def sinir_hucreleri(grid):
    """Frontier hücreleri: boş olan VE en az bir komşusu bilinmeyen hücreler."""
    bos = (grid >= 0 ) & (grid < BOS_ESIK)
    return bos & komsu_var(grid == BILINMIYOR)


def kumele(maske):
    """Yan yana (8 komşulu) True hücreleri gruplar. Her grup: (N, 2) [satır, sütun] dizisi."""
    h, w = maske.shape
    ziyaret = np.zeros_like(maske, dtype=bool)
    kumeler = []
    for r0, c0 in zip(*np.nonzero(maske)):
        if ziyaret[r0, c0]:
            continue
        ziyaret[r0, c0] = True
        kuyruk = deque([(r0, c0)])
        uyeler = []
        while kuyruk:
            r, c = kuyruk.popleft()
            uyeler.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < h and 0 <= cc < w and maske[rr, cc] and not ziyaret[rr, cc]:
                        ziyaret[rr, cc] = True
                        kuyruk.append((rr, cc))
        kumeler.append(np.array(uyeler))
    return kumeler

def genislet(maske, r):
    """True bölgeleri r hücre yarıçaplı bir daire kadar büyütür (dilation)."""
    if r <= 0:
        return maske.copy()
    h, w = maske.shape
    p = np.pad(maske, r, constant_values=False)
    sonuc = np.zeros_like(maske)
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dy * dy + dx * dx <= r * r:
                sonuc |= p[r + dy:r + dy + h, r + dx:r + dx + w]
    return sonuc

def sinirlari_bul(grid, cozunurluk, engel_payi_m=0.2, min_boyut_m=0.3):
    """Robotun gidebileceği frontier hedefleri.

    Dönüş: [((satir, sutun), boyut_m), ...]
    """
    dolu = grid >= DOLU_ESIK
    r = max(1, int(round(engel_payi_m / cozunurluk)))
    maske = sinir_hucreleri(grid) & ~genislet(dolu, r)
    sonuc = []
    for kume in kumele(maske):
        boyut_m = len(kume) * cozunurluk
        if boyut_m < min_boyut_m:
            continue
        merkez = kume.mean(axis=0)
        i = int(np.argmin(((kume - merkez) ** 2).sum(axis=1)))
        sonuc.append(((int(kume[i, 0]), int(kume[i, 1])), boyut_m))
    return sonuc

def bilinmeyen_kaldi_mi(grid,satir,sutun,yaricap):
    """Hücrenin çevresinde (yaricap hücre) hâlâ bilinmeyen alan var mı?"""
    h,w = grid.shape
    r0,r1 = max(0, satir - yaricap), min(h, satir + yaricap + 1)
    c0,c1 = max(0, sutun - yaricap), min(w,sutun + yaricap + 1)
    return bool((grid[r0:r1, c0:c1] == BILINMIYOR).any())