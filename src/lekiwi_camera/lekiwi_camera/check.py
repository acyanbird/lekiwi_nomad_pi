#!/usr/bin/env python3
import rclpy

def main(args=None):
    rclpy.init(args=args)
    print("Image viewer node started successfully!")
    # Intentionally keeping it minimal, no Node class or spinning
    rclpy.shutdown()

if __name__ == '__main__':
    main()