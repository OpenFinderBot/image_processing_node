# Image Processing Node

A Dockerized ROS 2 Python package for real-time image processing operations including distortion removal and region of interest (ROI) cropping.

## Features

- **Distortion Removal**: Removes lens distortion using camera calibration parameters
- **ROI Cropping**: Crops a specified region of interest from the image
- **Optimized Processing**: Uses pre-computed undistortion maps for efficient real-time processing
- **Dockerized Deployment**: Ready-to-run Docker container with all dependencies included

## Topics

### Subscribed Topics

- `image_raw` - Raw camera images (sensor_msgs/Image)
- `camera_info` - Camera calibration info (sensor_msgs/CameraInfo)

### Published Topics

- `image_clean` - Processed images (sensor_msgs/Image)

## Docker Installation

### Prerequisites

- Docker installed on your system
- ROS 2 network accessible from Docker container

### Build Docker Image

```bash
cd /path/to/image_processing_node
docker build -t image_processing_node .
```

Publishing to Docker repo:
```
docker buildx build \
  --platform linux/arm64/v8 \
  -t mykingdomisabsolute/open_finder_bot_image_processing_node:latest \
  --push \
  .
```

### Run with Docker

```bash
docker run --rm -it \
  --network host \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} \
  -e IMAGE_PROCESSING_NODE_NAMESPACE="/camera" \
  -e IMAGE_PROCESSING_NODE_PARAMETERS="/path/to/params.yaml" \
  image_processing_node
```

## Configuration

### Environment Variables

- `IMAGE_PROCESSING_NODE_NAMESPACE`: ROS namespace for the node (default: from environment)
- `IMAGE_PROCESSING_NODE_PARAMETERS`: Path to parameter file (default: from environment)
- `ROS_DOMAIN_ID`: ROS 2 domain ID for network isolation

### Parameter File

The node expects camera calibration parameters to be provided via the `camera_info` topic. ROI parameters are extracted from the CameraInfo message's ROI field.

## Testing

On the src folder inside a DevContainer:
  - source /opt/ros/${ROS_DISTRO}/setup.sh
  - colcon build && . install/setup.bash
  - colcon test --event-handlers console_cohesion+ --packages-select image_processing_node

## License

MIT License - See LICENSE file for details
