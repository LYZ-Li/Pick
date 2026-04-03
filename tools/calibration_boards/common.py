#!/usr/bin/env python3
import io
from pathlib import Path

import cv2
import yaml
from reportlab.lib.pagesizes import A4, letter
from reportlab.lib.utils import ImageReader


PAGE_SIZES = {
    'letter': letter,
    'a4': A4,
}

MM_TO_POINTS = 72.0 / 25.4


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def default_config_path() -> Path:
    return (
        repo_root()
        / 'ros2_ws'
        / 'src'
        / 'ur5e_pick_place_bringup'
        / 'config'
        / 'phase_b_calibration.yaml'
    )


def load_board_config(config_path: Path) -> dict:
    with config_path.open('r', encoding='utf-8') as file_handle:
        return yaml.safe_load(file_handle)


def default_output_dir() -> Path:
    return Path(__file__).resolve().parent / 'output'


def ensure_output_parent(output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)


def mm_to_points(value_mm: float) -> float:
    return value_mm * MM_TO_POINTS


def meters_to_mm(value_m: float) -> float:
    return value_m * 1000.0


def get_dictionary(dictionary_name: str):
    try:
        dictionary_id = getattr(cv2.aruco, dictionary_name)
    except AttributeError as exc:
        raise ValueError(f'Unsupported ArUco dictionary: {dictionary_name}') from exc
    return cv2.aruco.getPredefinedDictionary(dictionary_id)


def draw_marker_image(dictionary, marker_id: int, image_size_px: int):
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, image_size_px)

    marker_image = 255 * cv2.UMat(image_size_px, image_size_px, cv2.CV_8UC1).get()
    cv2.aruco.drawMarker(dictionary, marker_id, image_size_px, marker_image, 1)
    return marker_image


def image_to_reader(image) -> ImageReader:
    success, encoded = cv2.imencode('.png', image)
    if not success:
        raise RuntimeError('Failed to encode generated board image as PNG.')
    return ImageReader(io.BytesIO(encoded.tobytes()))


def create_charuco_board(
    dictionary,
    squares_x: int,
    squares_y: int,
    square_length_m: float,
    marker_length_m: float,
):
    if hasattr(cv2.aruco, 'CharucoBoard') and callable(
        getattr(cv2.aruco, 'CharucoBoard')
    ):
        return cv2.aruco.CharucoBoard(
            (squares_x, squares_y),
            square_length_m,
            marker_length_m,
            dictionary,
        )
    return cv2.aruco.CharucoBoard_create(
        squares_x,
        squares_y,
        square_length_m,
        marker_length_m,
        dictionary,
    )


def draw_charuco_board(board, image_width_px: int, image_height_px: int):
    board_size = (image_width_px, image_height_px)
    if hasattr(board, 'generateImage'):
        return board.generateImage(board_size)
    return board.draw(board_size)
