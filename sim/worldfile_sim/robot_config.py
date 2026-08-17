"""Robot-owned configuration carried through MuJoCo's compiled model."""

import mujoco


def custom_text(model, key):
    """Return one <custom><text> value, naming the missing key on failure."""
    text_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXT, key)
    if text_id < 0:
        raise ValueError(f"robot config is missing custom text '{key}'")
    start = int(model.text_adr[text_id])
    size = int(model.text_size[text_id])
    raw = bytes(model.text_data[start:start + size])
    return raw.split(b"\0", 1)[0].decode("utf-8")


def scan_site(model, robot):
    """Return (site name, site id) declared by <robot>.scan_site."""
    key = f"{robot}.scan_site"
    name = custom_text(model, key)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    if site_id < 0:
        raise ValueError(
            f"robot config '{key}' names missing site '{name}'")
    return name, site_id
