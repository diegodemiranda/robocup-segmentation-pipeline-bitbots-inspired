# Segmentation Pipeline — Field Boundary, Obstacle, and Line Detection

A modular computer vision pipeline for RoboCup humanoid soccer, implementing classical image processing techniques for field boundary detection, obstacle classification, and field marking point detection using HSV color segmentation and OpenCV.

This project is inspired by *An Open Source Vision Pipeline Approach for RoboCup Humanoid Soccer* by Fiedler, Brandt, Gutsche, Vahl, Hagge, and Bestmann from the Hamburg Bit-Bots team. It follows the **pipe-and-filter architecture**, in which each module performs an independent processing task and produces structured outputs for subsequent components.

The implementation is intended for experimentation, algorithm development, and integration into larger robotic perception systems.

## Related Projects

### RoboCup Vision Pipeline

**Repository:** [robocup-vision-pipeline](https://github.com/diegodemiranda/robocup-vision-pipeline)

For a broader computer vision system integrating neural object detection and robotics middleware, see [RoboCup Vision Pipeline](https://github.com/diegodemiranda/robocup-vision-pipeline).

The two projects address complementary aspects of visual perception for RoboCup humanoid soccer:

- **Segmentation Pipeline (this repository):** focuses on classical image processing algorithms, including HSV color segmentation, field boundary detection, obstacle classification, and field marking point sampling.
- **RoboCup Vision Pipeline:** focuses on integrating YOLO-based object detection, detection validation, image processing, and ROS 2 communication within a Webots-based robotics workflow.

This repository can be explored and tested independently of the broader vision system. The projects are maintained separately to preserve the modularity and reusability of the segmentation algorithms.

The existence of the related project does not imply a direct software dependency: integration depends on the interfaces and data structures used by each component.

## Architecture

The pipeline uses a modular pipe-and-filter design. The shared color processing module provides HSV masks to the individual detectors, which can then be orchestrated by the main pipeline.

```text
                  ┌────────────────────┐
      Image ────▶ │  color_common.py   │
                  └─────────┬──────────┘
                            │
             ┌──────────────┼──────────────┐
             ▼              ▼              ▼
      field_boundary_  obstacle_      line_detector.py
      detector.py      detector.py
             │              │              │
             └──────────────┴──────────────┘
                            │
                            ▼
                     main_pipeline.py
                  Orchestration and debug
```

## Features

- HSV-based color segmentation with configurable color ranges.
- Field boundary detection using column-wise scanning.
- Convex hull computation to estimate the outer field boundary.
- Obstacle candidate extraction using the difference between the estimated field region and its detected boundary.
- Obstacle classification based on average color.
- Field marking point sampling using a binary marking mask.
- Optional sampling bias toward lower image regions and previously detected points.
- Debug image generation for visual inspection of detections.
- CSV-based configuration of HSV color ranges.

## Modules

### `color_common.py`

Defines the shared color representation and segmentation utilities.

**`ColorSpaceHSV`**

A dataclass containing the minimum and maximum HSV values for a color range. Its `mask(image)` method returns a binary mask identifying pixels within the configured range.

**`ColorDetector`**

Groups the five color spaces used by the pipeline:

- `field`: green field surface.
- `marking`: white field markings.
- `red`: red team markers.
- `blue`: blue team markers.
- `white`: white goal structures.

It exposes methods such as `field_mask()` and `marking_mask()` for use by the detection modules.

**`DEFAULT_COLOR_SPACES`**

Provides example HSV values for initial testing. These values are placeholders and should be calibrated for the actual camera, lighting, and environment.

**`load_color_spaces_from_csv(csv_path)`**

Loads HSV ranges from a CSV file and converts them into `ColorSpaceHSV` objects.

The expected CSV columns are:

`classe,h_min,h_max,s_min,s_max,v_min,v_max`

**`build_color_detector(color_spaces)`**

Constructs a `ColorDetector` from a dictionary of HSV color ranges, regardless of whether the values come from the defaults or a CSV file.

This module corresponds conceptually to the Color Detector component described in Section 3.1 of the referenced paper. Unlike the paper's adaptive lookup-table approach for field color detection, this implementation uses configurable HSV ranges for all supported color classes.

### `field_boundary_detector.py`

Main function:

```python
detect_field_boundary(
    image_bgr,
    color_detector,
    column_step,
    search_from_top,
    kernel_size
)
```

Detects the field boundary using column-wise scanning rather than evaluating every pixel independently.

Two scanning modes are available:

- **`search_from_top=True`:** scans each column from top to bottom until a green field pixel is found. This mode is intended for views in which the field occupies most of the image.
- **`search_from_top=False`:** scans from bottom to top, using a morphological kernel to reduce confusion between internal white markings and the external field boundary. This mode is intended for camera views with a more upward-facing orientation.

The detector then computes a convex hull from the detected points to reduce irregularities caused by obstacles occluding the field boundary.

Returns:

`(boundary_points, hull_mask)`

The detected points and hull mask are subsequently used by the obstacle and line detection modules.

### `obstacle_detector.py`

Main function:

```python
detect_obstacles(
    image_bgr,
    color_detector,
    field_boundary_points,
    hull_mask,
    min_area
)
```

Identifies candidate obstacles using the difference between the convex hull representing the estimated field region and the detected field boundary.

Connected components in the resulting difference region are filtered by their area. Candidates exceeding `min_area` are classified according to the average color inside their bounding boxes.

The current implementation uses color predominance to distinguish candidate classes:

- White: goal structure candidate.
- Red: red team robot candidate.
- Blue: blue team robot candidate.

Returns a list of `Obstacle` objects.

Each `Obstacle` contains:

- `x`
- `y`
- `width`
- `height`
- `color_class`

The `.bbox` property provides the bounding-box representation.

**Note:** these are heuristic classifications based on color and geometry, not semantic classifications produced by a trained neural network.

### `line_detector.py`

Main function:

```python
detect_line_points(
    image_bgr,
    color_detector,
    field_boundary_points,
    n_samples,
    rng,
    previous_detections,
    density_boost_radius
)
```

Rather than extracting complete line geometries, this module samples image points that belong to white field markings.

This design follows the point-based approach described in the reference paper, in which sampled marking points can provide observations for robot localization without requiring full geometric reconstruction of every line.

Sampling is biased toward:

- Lower image regions, where field markings are generally closer to the robot.
- Regions near detections from the previous frame, approximating temporal continuity.

A sampled point is accepted only if it falls within the marking mask.

The resulting points can be used by downstream localization algorithms, provided the expected coordinate conventions and data structures are compatible.

### `main_pipeline.py`

Coordinates the field boundary, obstacle, and line detection modules.

The main pipeline combines their outputs and generates a debug image named `debug_output.jpg`, providing a visual representation of the detected field boundary, sampled marking points, and obstacle bounding boxes.

This module serves as the entry point for running the segmentation pipeline, subject to the interface exposed by its implementation.

## Configuration

### Using default HSV values

The pipeline includes example HSV values for initial testing:

```python
color_spaces = DEFAULT_COLOR_SPACES
```

These values should not be assumed to work reliably under different lighting conditions, cameras, or simulation environments.

### Loading calibrated HSV values from CSV

For calibrated color ranges, load the values from a CSV file:

```python
color_spaces = load_color_spaces_from_csv("my_hsv_values.csv")
```

The CSV file must contain one row per color class and the following columns:

```csv
classe,h_min,h_max,s_min,s_max,v_min,v_max
field,35,85,50,255,30,255
marking,0,180,0,60,180,255
red,0,10,100,255,80,255
blue,100,130,100,255,80,255
white,0,180,0,60,180,255
```

The values above are examples only. They must be calibrated for the actual image source and environment.

The CSV loading operation is implemented in `load_color_spaces_from_csv()` in `color_common.py`.

## Installation

The implementation uses Python, OpenCV, NumPy, and pandas.

Install the dependencies in your preferred Python environment:

```bash
python -m pip install numpy opencv-python pandas
```

Clone the repository:

```bash
git clone https://github.com/diegodemiranda/robocup-segmentation-pipeline-bitbots-inspired.git
cd robocup-segmentation-pipeline-bitbots-inspired
```

The commands above assume that the repository is publicly available at the specified URL and that its files are located at the repository root.

## Running the Pipeline

The main entry point is `main_pipeline.py`.

```bash
python main_pipeline.py
```

This command assumes that the script's image input and configuration are already set up as expected by the implementation. Configure the input image or camera source and HSV values before running it in a different environment.

If the pipeline completes successfully, the current implementation generates a debug image named `debug_output.jpg`.

The precise runtime arguments and input configuration depend on the code in `main_pipeline.py`.

## ROS Integration

This repository currently describes a standalone image-processing pipeline. Its modules can serve as building blocks for a ROS 2 perception node, but ROS integration requires an additional adapter layer.

A possible integration architecture is:

1. Subscribe to a camera image topic, such as `/image_raw`, using `sensor_msgs/Image`.
2. Convert the incoming image into an OpenCV-compatible representation using `cv_bridge`.
3. Call the segmentation pipeline on each incoming frame.
4. Publish the resulting field boundary points, obstacle candidates, and marking points using suitable ROS 2 message types.
5. Optionally publish the debug image on a topic such as `/vision/debug_image`.

The output data structures must be mapped to the chosen ROS 2 message definitions. Custom messages may be required for obstacle bounding boxes and arrays of image points.

The detected marking points and field boundary may also be useful to a localization system, provided that the image observations are converted into the coordinate representation expected by that system.

These steps describe a possible integration path, not functionality that is necessarily implemented in this repository.

## Neural Network Integration

The referenced paper uses a fully convolutional neural network (FCNN) for ball detection alongside conventional image-processing modules.

A similar extension could introduce a dedicated detector, such as `ball_fcnn_detector.py`, while keeping the existing field boundary, obstacle, and line detection modules independent.

A potential implementation would:

1. Preprocess the input image according to the network's input requirements.
2. Run inference to obtain a heatmap or other model output.
3. Extract one or more ball candidates from the model output.
4. Convert the resulting detections into a representation compatible with the rest of the perception system.

The details depend on the selected model and its output format. Neural network ball detection is a potential extension and should not be considered an existing feature unless it has been implemented separately.

## Project Structure

The main modules and their responsibilities are summarized below.

| File | Responsibility | Main dependencies |
|---|---|---|
| `color_common.py` | HSV color spaces, masks, and color detector construction | NumPy, OpenCV, pandas |
| `field_boundary_detector.py` | Field boundary detection and convex hull computation | `color_common.py`, OpenCV |
| `obstacle_detector.py` | Obstacle candidate extraction and color-based classification | `color_common.py`, OpenCV |
| `line_detector.py` | Sampling points belonging to field markings | `color_common.py`, NumPy, OpenCV |
| `main_pipeline.py` | Pipeline orchestration and debug image generation | The modules above |

## Limitations

The current approach relies on classical image processing and heuristic rules. Its performance may vary with:

- Changes in lighting and shadows.
- Camera characteristics and color calibration.
- Occlusions and objects near the field boundary.
- Similarities between the colors of robots, field markings, and other objects.
- Camera orientation and the proportion of the image occupied by the field.

HSV thresholds should be calibrated for the target environment. The obstacle detector's color-based classification should not be interpreted as equivalent to learned object recognition.

The pipeline also does not, by itself, establish real-world field coordinates from image coordinates. This requires a compatible geometric projection or localization component.

## Reference

Fiedler, Brandt, Gutsche, Vahl, Hagge, and Bestmann. *An Open Source Vision Pipeline Approach for RoboCup Humanoid Soccer*. Hamburg Bit-Bots.

The implementation is inspired by the modular architecture and selected processing ideas described in this work. It should not be interpreted as a complete reproduction of the original system.

## License

Add a `LICENSE` file to this repository and specify the license under which the implementation is distributed.

Before choosing a license, verify the licensing terms applicable to the original reference material and any third-party code incorporated into this project.

## Contributing

Contributions that improve modularity, calibration, testing, performance, or integration with robotics systems are welcome.

When submitting changes, please:

1. Keep detection modules independent where practical.
2. Document changes to function signatures and output data structures.
3. Include tests or reproducible examples for new behavior.
4. Document any assumptions about image input, color calibration, or camera orientation.

## Acknowledgements

This project is inspired by the work of the Hamburg Bit-Bots team on modular vision pipelines for RoboCup humanoid soccer.

The related [RoboCup Vision Pipeline](https://github.com/diegodemiranda/robocup-vision-pipeline) explores a broader integration of neural object detection and robotics middleware.
