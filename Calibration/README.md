# HOLOLENS CALIBRATION
This folder contains all the useful codes to perform a camera calibration
For more information refers to [OpenCV Camera Calibration](https://docs.opencv.org/4.7.0/dc/dbb/tutorial_py_calibration.html)

## How to use:
- Prepare a chessboard marker for calibration printing in full scale the [chessboard.pdf](chessboard.pdf)
- Take at least 10 picture with your camera and move them inside the _"../CalibImage/"_ directory
- Run the python script
  ```bash
    python calibrate.py
  ```
  If necessary of you have a different type of chessboard you can change the following parameters:
  
    | Argument            | Description                                        | Required | Default |
    |---------------------|----------------------------------------------------|----------|---------|
    | `--debug`           | It enable debug and create an _"/output/"_: folder | `False`  | `False` |
    | `--square_size`     | Chessboard squares size in cm                      | `False`  | `1`     |
    | `--pattern_width`   | Chessboard pattern width  cm                       | `False`  | `7`     |
    | `--pattern_height`  | Chessboard pattern height in cm                    | `False`  | `5`     |

    ```bash
    pythpm Calibration/calibrate.py [--debug <output path>] [--square_size] [--pattern_width] [--pattern_height] [<image mask>]
    ```
- The script will create a _.json_ and _.yaml_ file containg all the intrinsc parameters of the camera 
