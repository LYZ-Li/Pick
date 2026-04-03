# Calibration Board Tools

This folder contains printable board generators for Phase B calibration.

Default paper size is US Letter.

## Requirements

- Python 3
- OpenCV with the `aruco` module
- `reportlab`
- `PyYAML`

Inside the project Docker image, these dependencies are installed by the updated Dockerfile.

For a local Python environment:

```bash
python3 -m pip install opencv-contrib-python reportlab pyyaml
```

## Default Outputs

Running the scripts without extra arguments creates:

- `tools/calibration_boards/output/aruco_board_letter.pdf`
- `tools/calibration_boards/output/charuco_board_letter.pdf`

## ArUco Board

This is the default printable target for the Phase B runtime calibration path.

```bash
python3 tools/calibration_boards/generate_aruco_board_pdf.py
```

The default board values come from:

`ros2_ws/src/ur5e_pick_place_bringup/config/phase_b_calibration.yaml`

Key defaults:

- dictionary: `DICT_ARUCO_ORIGINAL`
- board layout: `3 x 4`
- marker size: `40 mm`
- marker separation: `10 mm`
- first marker id: `0`
- tracked marker id for Phase B: `4`

Example override:

```bash
python3 tools/calibration_boards/generate_aruco_board_pdf.py \
  --page-size letter \
  --columns 4 \
  --rows 5 \
  --marker-size-mm 35 \
  --marker-separation-mm 8
```

## ChArUco Board

This tool is provided for printing and future calibration workflows. It is not the default Phase B runtime target.

```bash
python3 tools/calibration_boards/generate_charuco_board_pdf.py
```

Default values:

- dictionary: `DICT_ARUCO_ORIGINAL`
- squares: `5 x 7`
- square length: `30 mm`
- marker length: `23 mm`

Example override:

```bash
python3 tools/calibration_boards/generate_charuco_board_pdf.py \
  --page-size letter \
  --squares-x 6 \
  --squares-y 8 \
  --square-length-mm 28 \
  --marker-length-mm 20
```

## Printing Guidance

- Print at 100% scale.
- Disable page scaling or "fit to page" in the print dialog.
- Measure the printed marker or square size with a ruler before calibration.
- Mount the printed page on a rigid, flat backing board.
