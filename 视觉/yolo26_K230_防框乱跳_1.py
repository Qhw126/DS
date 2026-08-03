from libs.PipeLine import PipeLine, ScopedTiming
# from libs.YOLO import YOLOv8
from libs.YOLO import YOLO11
from libs.Utils import *
from media.sensor import *
import os, sys, gc
import ulab.numpy as np
import image
import time

if __name__ == "__main__":
    # ============================================================================
    # 1. 核心参数配置
    # ============================================================================
    rgb888p_size = [640, 480]
    display_size = [800, 480]
    model_input_size = [320, 320]

    kmodel_path = "/sdcard/best.kmodel"
    labels = ['ball']
    # 【修改 1】将置信度从 0.05 调高到 0.5，过滤掉背景噪点引起的乱跳
    confidence_threshold = 0.6

    # ============================================================================
    # 2. 硬件与 Pipeline 初始化
    # ============================================================================
    sensor = Sensor(width=1280, height=960)
    pl = PipeLine(rgb888p_size=rgb888p_size, display_size=display_size, display_mode="lcd")
    pl.create(sensor=sensor)

    # ============================================================================
    # 3. YOLO 模型初始化
    # ============================================================================
    yolo = YOLO11(task_type="detect", mode="video", kmodel_path=kmodel_path, labels=labels,
                  rgb888p_size=rgb888p_size, model_input_size=model_input_size,
                  display_size=display_size, conf_thresh=confidence_threshold, debug_mode=0)
    yolo.config_preprocess()

    print("🚀 核心识图程序已启动...")

    # ============================================================================
    # 4. 主程序循环
    # ============================================================================
    clock = time.clock()

    # 【修改 2】定义多帧防抖状态变量
    detect_count = 0
    missing_count = 0
    is_visible = False
    last_res = None

    try:
        while True:
            os.exitpoint()

            clock.tick()

            # 1. 获取摄像头当前帧图像
            img = pl.get_frame()

            # 2. 将图像送入模型进行推理，返回结果
            res = yolo.run(img)

            # 【修改 3】加入连续 3 帧判定防抖逻辑
            if res and len(res) > 0:
                missing_count = 0
                detect_count += 1
                if detect_count >= 3:  # 连续 3 帧检测到才判定为真实目标
                    is_visible = True
                    detect_count = 3
                last_res = res  # 暂存当前有效结果
            else:
                detect_count = 0
                missing_count += 1
                if missing_count >= 3:  # 连续 3 帧消失才真正取消显示
                    is_visible = False
                    missing_count = 3
                    last_res = None

            # 3. 清屏并根据防抖状态调用官方绘制功能，将检测框画在 OSD 涂层上
            pl.osd_img.clear()
            if is_visible and last_res is not None:
                yolo.draw_result(last_res, pl.osd_img)

            # 4. 计算 FPS 并使用 draw_string_advanced 绘制在屏幕左上角
            fps = clock.fps()
            pl.osd_img.draw_string_advanced(10, 10, 30, f"FPS: {fps:.1f}", color=(255, 0, 0))

            # 5. 刷新屏幕显示画面
            pl.show_image()

            # 6. 垃圾回收，防止内存溢出
            gc.collect()

    except Exception as e:
        sys.print_exception(e)
    finally:
        # 释放 KPU 资源与 Pipeline 资源
        yolo.deinit()
        pl.destroy()
        print("🛑 程序已安全退出并释放资源。")
