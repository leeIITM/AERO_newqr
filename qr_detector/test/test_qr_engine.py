import cv2
import numpy as np

from qr_detector.qr_engine import QrFinder, compute_similarity, extract_qr_features, match


def code_image(text, rot=0, module=10, canvas=600):
    code = cv2.QRCodeEncoder.create().encode(text)
    code = cv2.resize(code, (code.shape[1] * module, code.shape[0] * module), interpolation=cv2.INTER_NEAREST)
    code = np.rot90(code, rot).copy()
    img = np.full((canvas, canvas), 255, np.uint8)
    o = (canvas - code.shape[0]) // 2
    img[o:o + code.shape[0], o:o + code.shape[1]] = code
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def features(img):
    ok, pts, gray, text = QrFinder().detect(img)
    assert ok
    grid, _ = extract_qr_features(gray, pts)
    return grid, text


def test_rotation_invariant_similarity():
    ref, t_ref = features(code_image("AEROTHON-7"))
    for rot in range(4):
        cand, t = features(code_image("AEROTHON-7", rot))
        assert t == t_ref == "AEROTHON-7"
        assert compute_similarity(ref, cand) > 0.99


def test_text_mismatch_rejected():
    ref, t_ref = features(code_image("AEROTHON-7"))
    other, t_other = features(code_image("AEROTHON-3"))
    score, matched = match(ref, t_ref, other, t_other, 0.88)
    assert not matched
    # the original rule (grid score only) would have accepted this decoy
    _, matched_original = match(ref, t_ref, other, t_other, 0.88, reject_text_mismatch=False)
    assert score > 0.88 and matched_original


def test_text_match_and_undecoded_grid_match():
    ref, t_ref = features(code_image("AEROTHON-7"))
    cand, t = features(code_image("AEROTHON-7", 1))
    assert match(ref, t_ref, cand, t, 0.88) == (1.0, True)
    score, matched = match(ref, t_ref, cand, "", 0.88)  # candidate not decodable: grid decides
    assert matched and score > 0.99
