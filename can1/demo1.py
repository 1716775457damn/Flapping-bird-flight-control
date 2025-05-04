import cv2
import numpy as np
import pyrealsense2 as rs


def main():
    # 创建RealSense管线对象
    pipeline = rs.pipeline()
    config = rs.config()

    # 配置流
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # 开始流
    pipeline.start(config)

    # 创建对齐对象
    align = rs.align(rs.stream.color)

    try:
        while True:
            # 等待一组连贯的帧: 深度帧和彩色帧
            frames = pipeline.wait_for_frames()

            # 对齐帧
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # 转换为numpy数组
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # 应用彩色映射到深度图像
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
            )

            # 水平堆叠图像
            images = np.hstack((color_image, depth_colormap))

            # 显示图像
            cv2.namedWindow("RealSense", cv2.WINDOW_AUTOSIZE)
            cv2.imshow("RealSense", images)

            # 按'q'退出
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        # 停止流
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
