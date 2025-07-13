#!/usr/bin/env python

import os
import glob
import subprocess
import argparse
import tempfile
from PIL import Image
import numpy as np

def get_pgm_dimensions(pgm_file):
    """Get the dimensions of a PGM file."""
    with Image.open(pgm_file) as img:
        return img.size  # (width, height)

def save_pgm_p5(filename, image_array):
    """Save a numpy array as a P5 (binary) PGM file."""
    height, width = image_array.shape
    with open(filename, 'wb') as f:
        # Write PGM P5 header
        f.write(b'P5\n')
        f.write(f'{width} {height}\n'.encode('ascii'))
        f.write(b'255\n')  # Max value for 8-bit grayscale
        # Write binary pixel data
        image_array.tofile(f)
        
def pad_pgm(input_file, output_file, target_width, target_height, pad_value=255):
    """Pad a PGM file to target dimensions, centering the original content."""
    with Image.open(input_file) as img:
        img_array = np.array(img)
        
        # Current dimensions
        curr_height, curr_width = img_array.shape
        
        # Create a new array with target dimensions filled with pad_value
        padded_array = np.full((target_height, target_width), pad_value, dtype=np.uint8)
        
        # Calculate offsets to center the original image
        x_offset = (target_width - curr_width) // 2
        y_offset = (target_height - curr_height) // 2
        
        # Place the original image in the center
        padded_array[y_offset:y_offset+curr_height, x_offset:x_offset+curr_width] = img_array
        
        # Save as P5 PGM
        save_pgm_p5(output_file, padded_array)

def create_video(input_dir, output_file, fps=5):
    # Get all PGM files in the input directory
    pgm_files = sorted(glob.glob(os.path.join(input_dir, "*.pgm")))
    
    if not pgm_files:
        print(f"No PGM files found in {input_dir}")
        return
    
    # Find maximum dimensions among all PGM files
    max_width = 0
    max_height = 0
    for pgm_file in pgm_files:
        width, height = get_pgm_dimensions(pgm_file)
        max_width = max(max_width, width)
        max_height = max(max_height, height)
    
    # Create a temporary directory for padded PGM files
    with tempfile.TemporaryDirectory() as temp_dir:
        padded_files = []
        for i, pgm_file in enumerate(pgm_files):
            padded_file = os.path.join(temp_dir, f"padded_{i:06d}.pgm")
            pad_pgm(pgm_file, padded_file, max_width, max_height, pad_value=255)
            padded_files.append(padded_file)
        
        # Create a temporary text file listing the padded PGM files
        temp_list_file = os.path.join(temp_dir, "pgm_list.txt")
        with open(temp_list_file, "w") as f:
            for padded_file in padded_files:
                f.write(f"file '{padded_file}'\nduration {1.0/fps}\n")
        
        try:
            # Construct ffmpeg command for AVI output with mpeg4 codec
            ffmpeg_cmd = [
                "ffmpeg",
                "-f", "concat",
                "-safe", "0",
                "-i", temp_list_file,
                "-c:v", "mpeg4",
                "-r", str(fps),
                "-q:v", "2",  # High quality for mpeg4
                output_file
            ]
            
            # Run ffmpeg command
            result = subprocess.run(ffmpeg_cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                print(f"Video created successfully: {output_file}")
            else:
                print(f"Error creating video: {result.stderr}")
                
        except Exception as e:
            print(f"Error running ffmpeg: {e}")

if __name__ == "__main__":
    # Get the directory of the script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    parser = argparse.ArgumentParser(description="Create a video from PGM files using ffmpeg with padding to max size")
    parser.add_argument("--input_dir", default=script_dir, help="Directory containing PGM files (default: script's directory)")
    parser.add_argument("--output_file", default="mapping_procedure.avi", help="Output video file name")
    parser.add_argument("--fps", type=int, default=5, help="Frames per second for the output video")
    
    args = parser.parse_args()
    
    create_video(args.input_dir, args.output_file, args.fps)