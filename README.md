# Dual ROI ZWO Camera Viewer

Live camera viewer for ZWO ASI cameras with two configurable regions of interest (ROI). Each ROI can be independently moved, resized, rotated, and mirrored. Built with the ZWO ASI Camera SDK and OpenCV.

## Features

- Live video feed from any ZWO ASI camera
- Two independent ROIs with colored overlays
- Per-ROI rotation (90°/180°/270°) and mirroring (horizontal/vertical)
- Adjustable exposure via keyboard
- Save ROI snapshots to disk
- Interactive control panel with clickable buttons
- Mouse-driven ROI positioning and resizing

## Prerequisites

- Linux (x64)
- g++
- OpenCV 4
- A ZWO ASI camera (e.g. ASI662MC)

### Install OpenCV (Ubuntu/Debian)

```bash
sudo apt install libopencv-dev pkg-config
```

## Installation

```bash
git clone https://github.com/chengyi33/dualroi_zwo.git
cd dualroi_zwo
```

### Set up udev rules (required for camera access without root)

```bash
make install-udev
```

Then **unplug and replug** your camera.

### Build

```bash
make
```

## Usage

```bash
make run
```

Or directly:

```bash
LD_LIBRARY_PATH=./lib/x64 ./bin/dual_roi
```

### Keyboard Controls

| Key | Action |
|-----|--------|
| `1` | Select ROI 1 |
| `2` | Select ROI 2 |
| `R` | Rotate active ROI 90° clockwise |
| `H` | Mirror active ROI horizontally |
| `V` | Mirror active ROI vertically |
| `S` | Save both ROI snapshots |
| `+` / `=` | Increase exposure |
| `-` / `_` | Decrease exposure |
| `Q` / `Esc` | Quit |

### Mouse Controls

- **Click & drag** on the main view to move the active ROI
- Use the **control panel window** for clickable buttons

### Windows

- **Main View** — full camera frame with ROI overlays
- **ROI 1** / **ROI 2** — cropped and transformed ROI views
- **Controls** — interactive button panel

## Project Structure

```
├── src/
│   └── dual_roi.cpp    # Main application
├── include/
│   └── ASICamera2.h    # ZWO ASI SDK header
├── lib/
│   └── x64/            # ZWO ASI SDK library (Linux x64)
├── udev/
│   └── asi.rules       # udev rules for camera permissions
├── bin/                 # Build output
├── Makefile
└── README.md
```

## License

ZWO ASI Camera SDK is property of ZWO. See [ZWO SDK](https://www.zwoastro.com/software/) for their terms.
