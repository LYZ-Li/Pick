#!/usr/bin/env python3
import argparse
from pathlib import Path

from reportlab.pdfgen import canvas

from common import (
    PAGE_SIZES,
    create_charuco_board,
    default_config_path,
    default_output_dir,
    draw_charuco_board,
    ensure_output_parent,
    get_dictionary,
    image_to_reader,
    load_board_config,
    meters_to_mm,
    mm_to_points,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Generate a printable ChArUco board PDF.'
    )
    parser.add_argument(
        '--config',
        type=Path,
        default=default_config_path(),
        help='Path to the board YAML config file.',
    )
    parser.add_argument(
        '--page-size',
        choices=sorted(PAGE_SIZES.keys()),
        default='letter',
        help='PDF page size.',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=default_output_dir() / 'charuco_board_letter.pdf',
        help='Output PDF path.',
    )
    parser.add_argument(
        '--squares-x',
        type=int,
        default=None,
        help='Number of squares along the x axis.',
    )
    parser.add_argument(
        '--squares-y',
        type=int,
        default=None,
        help='Number of squares along the y axis.',
    )
    parser.add_argument(
        '--square-length-mm',
        type=float,
        default=None,
        help='Square side length in millimeters.',
    )
    parser.add_argument(
        '--marker-length-mm',
        type=float,
        default=None,
        help='Marker side length in millimeters.',
    )
    parser.add_argument(
        '--margin-mm',
        type=float,
        default=12.7,
        help='Page margin in millimeters.',
    )
    parser.add_argument(
        '--dictionary',
        default=None,
        help='OpenCV ArUco dictionary name.',
    )
    return parser.parse_args()


def main():
    args = parse_args()
    config = load_board_config(args.config)['charuco_board']

    squares_x = args.squares_x or config['squares_x']
    squares_y = args.squares_y or config['squares_y']
    square_length_mm = args.square_length_mm or meters_to_mm(config['square_length_m'])
    marker_length_mm = args.marker_length_mm or meters_to_mm(config['marker_length_m'])
    dictionary = get_dictionary(args.dictionary or config['dictionary'])

    page_width, page_height = PAGE_SIZES[args.page_size]
    margin_pt = mm_to_points(args.margin_mm)
    board_width_pt = mm_to_points(squares_x * square_length_mm)
    board_height_pt = mm_to_points(squares_y * square_length_mm)

    if (
        board_width_pt + 2 * margin_pt > page_width
        or board_height_pt + 2 * margin_pt > page_height
    ):
        raise ValueError('Board dimensions exceed the selected page size and margin.')

    board = create_charuco_board(
        dictionary,
        squares_x,
        squares_y,
        square_length_mm / 1000.0,
        marker_length_mm / 1000.0,
    )
    pixel_width = max(int(round(square_length_mm * squares_x * 20.0)), 1200)
    pixel_height = max(int(round(square_length_mm * squares_y * 20.0)), 1200)
    board_image = draw_charuco_board(board, pixel_width, pixel_height)
    board_reader = image_to_reader(board_image)

    origin_x = (page_width - board_width_pt) / 2.0
    origin_y = (page_height - board_height_pt) / 2.0

    ensure_output_parent(args.output)
    pdf = canvas.Canvas(str(args.output), pagesize=(page_width, page_height))
    pdf.drawImage(
        board_reader,
        origin_x,
        origin_y,
        width=board_width_pt,
        height=board_height_pt,
        preserveAspectRatio=True,
        mask='auto',
    )
    pdf.showPage()
    pdf.save()


if __name__ == '__main__':
    main()
