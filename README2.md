# PAROL‑Commander‑Software – My Fork

A lightweight fork of the original **PAROL commander** project focused on cleaning up the architecture and making it easier to extend.

## What I changed

* **Isolated launch logic** – `main.py` now only starts/stops processes; all robot logic lives elsewhere.
* **Created `tools/init_tools.py`** – one helper module that:

  * resolves the `ImageGUI/` path for all platforms
  * opens the serial port once and shares it
* **Re‑organised file tree** so communication, GUI and simulator are clearly separated.
* Added comments / logging to make the code more readable.

That’s it – everything else behaves the same as the original project.
