#!/usr/bin/env python3
import argparse
from pathlib import Path

from reportlab.pdfgen import canvas

from common import (
    PAGE_SIZES,
    default_config_path,
    default_output_dir,
    draw_marker_image,
    ensure_output_parent,
    get_dictionary,
    image_to_reader,
    load_board_config,
    meters_to_mm,
    mm_to_points,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Generate a printable ArUco board PDF.'
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
        default=default_output_dir() / 'aruco_board_letter.pdf',
        help='Output PDF path.',
    )
    parser.add_argument(
        '--columns',
        type=int,
        default=None,
        help='Number of marker columns.',
    )
    parser.add_argument('--rows', type=int, default=None, help='Number of marker rows.')
    parser.add_argument(
        '--marker-size-mm',
        type=float,
        default=None,
        help='Marker side length in millimeters.',
    )
    parser.add_argument(
        '--marker-separation-mm',
        type=float,
        default=None,
        help='Separation between markers in millimeters.',
    )
    parser.add_argument(
        '--first-marker-id',
        type=int,
        default=None,
        help='Starting marker id for the board.',
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
    config = load_board_config(args.config)['aruco_board']

    columns = args.columns or config['columns']
    rows = args.rows or config['rows']
    marker_size_mm = args.marker_size_mm or meters_to_mm(config['marker_size_m'])
    marker_separation_mm = args.marker_separation_mm or meters_to_mm(
        config['marker_separation_m']
    )
    if args.first_marker_id is not None:
        first_marker_id = args.first_marker_id
    else:
        first_marker_id = config['first_marker_id']
    dictionary = get_dictionary(args.dictionary or config['dictionary'])

    page_width, page_height = PAGE_SIZES[args.page_size]
    margin_pt = mm_to_points(args.margin_mm)
    marker_size_pt = mm_to_points(marker_size_mm)
    marker_separation_pt = mm_to_points(marker_separation_mm)
    board_width_pt = columns * marker_size_pt + (columns - 1) * marker_separation_pt
    board_height_pt = rows * marker_size_pt + (rows - 1) * marker_separation_pt

    if (
        board_width_pt + 2 * margin_pt > page_width
        or board_height_pt + 2 * margin_pt > page_height
    ):
        raise ValueError('Board dimensions exceed the selected page size and margin.')

    origin_x = (page_width - board_width_pt) / 2.0
    origin_y = (page_height - board_height_pt) / 2.0
    pixel_size = max(int(round(marker_size_mm * 20.0)), 300)

    ensure_output_parent(args.output)
    pdf = canvas.Canvas(str(args.output), pagesize=(page_width, page_height))

    marker_id = first_marker_id
    for row in range(rows):
        for column in range(columns):
            marker_image = draw_marker_image(dictionary, marker_id, pixel_size)
            marker_reader = image_to_reader(marker_image)
            x = origin_x + column * (marker_size_pt + marker_separation_pt)
            y = origin_y + (rows - 1 - row) * (marker_size_pt + marker_separation_pt)
            pdf.drawImage(
                marker_reader,
                x,
                y,
                width=marker_size_pt,
                height=marker_size_pt,
                preserveAspectRatio=True,
                mask='auto',
            )
            marker_id += 1

    pdf.showPage()
    pdf.save()


if __name__ == '__main__':
    main()
