import argparse, os, sys
import rclpy
from sensor_msgs.msg import Image

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--count', type=int, default=3)
    parser.add_argument('--out', default='/ros2_ws')
    parser.add_argument('--topic', default='/camera/color/image_raw')
    parser.add_argument('--prefix', default='snapshot')
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    node = rclpy.create_node('camera_capture')
    saved = []

    def on_image(msg: Image):
        if len(saved) >= args.count:
            return
        idx = len(saved) + 1
        filename = os.path.join(args.out, f'{args.prefix}_{idx:03d}.png')

        # msg.data is array.array('B',...) in ROS 2 — must convert to plain bytes
        raw = bytes(bytearray(msg.data))

        # Normalise to RGB regardless of camera encoding
        enc = msg.encoding.lower()
        if enc in ('bgr8', 'bgr'):
            ba = bytearray(raw)
            for i in range(0, len(ba), 3):
                ba[i], ba[i + 2] = ba[i + 2], ba[i]
            raw = bytes(ba)
        elif enc in ('mono8', '8uc1'):
            raw = bytes(b for byte in raw for b in (byte, byte, byte))
        # else assume rgb8

        try:
            from PIL import Image as PILImage
            img = PILImage.frombytes('RGB', (msg.width, msg.height), raw)
            img.save(filename)
        except Exception as e:
            node.get_logger().warn(f'PIL failed ({e}), writing PPM instead')
            filename = filename.replace('.png', '.ppm')
            with open(filename, 'wb') as f:
                f.write(f'P6\n{msg.width} {msg.height}\n255\n'.encode())
                f.write(raw)

        saved.append(filename)
        node.get_logger().info(f'Saved [{idx}/{args.count}]: {filename}')

    node.get_logger().info(f'Waiting for {args.count} frame(s) on {args.topic} ...')
    node.create_subscription(Image, args.topic, on_image, 10)
    while rclpy.ok() and len(saved) < args.count:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()
    if not saved:
        print('ERROR: no frames received — is the simulation running?', file=sys.stderr)
        sys.exit(1)
    print('\nSaved:', *saved, sep='\n  ')

if __name__ == '__main__':
    main()