import os
import cv2
import numpy as np
import time
from datetime import datetime, timedelta
from threading import Thread
import serial
from tflite_runtime.interpreter import Interpreter
import gpiod

# Initialize Serial Port
ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)
time.sleep(2)

# Initialize GPIOs
chip = gpiod.Chip("gpiochip4")
ir1 = chip.get_line(23)
ir2 = chip.get_line(24)
ir3 = chip.get_line(16)
ir4 = chip.get_line(25)

ir1.request(consumer="IR1", type=gpiod.LINE_REQ_DIR_IN)
ir2.request(consumer="IR2", type=gpiod.LINE_REQ_DIR_IN)
ir3.request(consumer="IR3", type=gpiod.LINE_REQ_DIR_IN)
ir4.request(consumer="IR4", type=gpiod.LINE_REQ_DIR_IN)


class VideoStream:
    def __init__(self, resolution=(640, 480), framerate=30, source=0):
        self.stream = cv2.VideoCapture(source)
        ret = self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        ret = self.stream.set(3, resolution[0])
        ret = self.stream.set(4, resolution[1])
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while True:
            if self.stopped:
                self.stream.release()
                return
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True


def ball():
    MODEL_NAME_1 = "/home/technotix/Robocon-2024/Robot 2 - Automatic/Ball Model"
    GRAPH_NAME_1 = "detect.tflite"
    LABELMAP_NAME_1 = "labelmap.txt"
    min_conf_threshold_1 = float(0.5)
    resW_1, resH_1 = 960, 540
    imW_1, imH_1 = int(resW_1), int(resH_1)
    use_TPU_1 = False
    intake = False
    ball_stored = False

    CWD_PATH = os.getcwd()
    PATH_TO_CKPT_1 = os.path.join(CWD_PATH, MODEL_NAME_1, GRAPH_NAME_1)
    PATH_TO_LABELS_1 = os.path.join(CWD_PATH, MODEL_NAME_1, LABELMAP_NAME_1)

    with open(PATH_TO_LABELS_1, "r") as f:
        labels_1 = [line.strip() for line in f.readlines()]

    if labels_1[0] == "???":
        del labels_1[0]

    interpreter_1 = Interpreter(model_path=PATH_TO_CKPT_1)

    interpreter_1.allocate_tensors()

    input_details_1 = interpreter_1.get_input_details()
    output_details_1 = interpreter_1.get_output_details()
    height_1 = input_details_1[0]["shape"][1]
    width_1 = input_details_1[0]["shape"][2]

    floating_model_1 = input_details_1[0]["dtype"] == np.float32
    input_mean_1 = 127.5
    input_std_1 = 127.5
    outname_1 = output_details_1[0]["name"]

    if "StatefulPartitionedCall" in outname_1:
        boxes_idx_1, classes_idx_1, scores_idx_1 = 1, 3, 0
    else:
        boxes_idx_1, classes_idx_1, scores_idx_1 = 0, 1, 2

    frame_rate_calc_1 = 1
    freq_1 = cv2.getTickFrequency()

    videostream_1 = VideoStream(
        resolution=(imW_1, imH_1), framerate=30, source=2
    ).start()
    time.sleep(1)

    center_line_1 = int(imW_1 // 2 - 150)
    center_line_2 = int(imW_1 // 2 + 150)
    line_height_1 = int(imH_1 - 200)
    mid_line_height_1 = int(imH_1 // 2) - 75

    tracker_1 = None
    tracking_1 = False
    track_box_1 = None
    current_label_1 = ""
    last_model_detect_1 = datetime.now() - timedelta(seconds=10)

    intake_time = datetime.now() - timedelta(seconds=10)

    while True:
        ir3_val = ir3.get_value()
        t1_1 = cv2.getTickCount()
        frame_1 = videostream_1.read()
        frame_rgb_1 = cv2.cvtColor(frame_1, cv2.COLOR_BGR2RGB)

        if intake and datetime.now() < intake_time + timedelta(seconds=3):
            print("Turn on Intake")
            ser.write("IZ".encode())
            cv2.putText(
                frame_1,
                "Intake",
                (50, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("Object Detector 1", frame_1)
            if cv2.waitKey(1) == ord("q"):
                break
        else:
            ser.write("OZ".encode())
            intake = False

        print(ir3.get_value())

        if ir3_val == 0:
            ser.write("SZ".encode())
            ser.write("OZ".encode())
            print("Break Hogaya")
            intake = False
            break

        if (
            tracking_1
            and last_model_detect_1 + timedelta(seconds=5) > datetime.now()
            and not intake
        ):
            success_1, track_box_1 = tracker_1.update(frame_1)
            if success_1:
                xmin_1, ymin_1 = track_box_1[0], track_box_1[1]
                xmax_1 = track_box_1[2] + xmin_1
                ymax_1 = track_box_1[3] + ymin_1
                p1_1 = (int(track_box_1[0]), int(track_box_1[1]))
                p2_1 = (
                    int(track_box_1[0] + track_box_1[2]),
                    int(track_box_1[1] + track_box_1[3]),
                )
                cv2.rectangle(frame_1, p1_1, p2_1, (255, 0, 0), 2, 1)
                box_center_x_1 = (xmin_1 + xmax_1) / 2
                box_center_y_1 = (ymin_1 + ymax_1) / 2

                if box_center_y_1 > mid_line_height_1:
                    center_line_1 = int(imW_1 // 2 - 150)
                    center_line_2 = int(imW_1 // 2 + 150)
                else:
                    center_line_1 = int(imW_1 // 2 - 100)
                    center_line_2 = int(imW_1 // 2 + 100)

                if (
                    box_center_y_1 > line_height_1
                    and not box_center_x_1 < center_line_1
                    and not box_center_x_1 > center_line_2
                ):
                    cv2.putText(
                        frame_1,
                        "Stop",
                        (50, 100),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    ser.write("IZ".encode())
                    intake = True
                    intake_time = datetime.now()
                elif box_center_x_1 < center_line_1:
                    cv2.putText(
                        frame_1,
                        "Left",
                        (50, 100),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    ser.write("LZ".encode())

                elif box_center_x_1 > center_line_2:
                    cv2.putText(
                        frame_1,
                        "Right",
                        (50, 100),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    ser.write("RZ".encode())

                elif center_line_1 < box_center_x_1 < center_line_2:
                    cv2.putText(
                        frame_1,
                        "Forward",
                        (50, 100),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    ser.write("FZ".encode())
                    cv2.putText(
                        frame_1,
                        "Center",
                        (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )

                else:
                    ser.write("IZ".encode())
                    ser.write("rZ".encode())

            else:
                ser.write("rZ".encode())
                ser.write("IZ".encode())
                tracking_1 = False
                last_model_detect_1 = datetime.now() - timedelta(seconds=10)

        else:
            image_rgb_1 = cv2.resize(frame_rgb_1, (width_1, height_1))
            input_data_1 = np.expand_dims(image_rgb_1, axis=0)
            if floating_model_1:
                input_data_1 = (np.float32(input_data_1) - input_mean_1) / input_std_1

            interpreter_1.set_tensor(input_details_1[0]["index"], input_data_1)
            interpreter_1.invoke()
            boxes_1 = interpreter_1.get_tensor(output_details_1[boxes_idx_1]["index"])[
                0
            ]
            classes_1 = interpreter_1.get_tensor(
                output_details_1[classes_idx_1]["index"]
            )[0]
            scores_1 = interpreter_1.get_tensor(
                output_details_1[scores_idx_1]["index"]
            )[0]

            print(scores_1)
            ball_detected = False

            for i in range(len(scores_1)):
                if (scores_1[i] > min_conf_threshold_1) and (scores_1[i] <= 1.0):
                    ball_detected = True
                    ymin_1 = int(max(1, (boxes_1[i][0] * imH_1)))
                    xmin_1 = int(max(1, (boxes_1[i][1] * imW_1)))
                    ymax_1 = int(min(imH_1, (boxes_1[i][2] * imH_1)))
                    xmax_1 = int(min(imW_1, (boxes_1[i][3] * imW_1)))
                    print(labels_1[int(classes_1[i])])

                    center_x_1 = (xmin_1 + xmax_1) / 2
                    center_y_1 = (ymin_1 + ymax_1) / 2

                    if center_x_1 > center_line_1 and center_x_1 < center_line_2:
                        tracker_1 = cv2.legacy.TrackerCSRT_create()
                        track_box_1 = (xmin_1, ymin_1, xmax_1 - xmin_1, ymax_1 - ymin_1)
                        tracker_1.init(frame_1, track_box_1)
                        tracking_1 = True
                        last_model_detect_1 = datetime.now()
            if not ball_detected:
                ser.write("IZ".encode())
                ser.write("rZ".encode())

        cv2.putText(
            frame_1,
            "FPS: {0:.2f}".format(frame_rate_calc_1),
            (30, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow("Object Detector 1", frame_1)

        if cv2.waitKey(1) == ord("q"):
            break

        t2_1 = cv2.getTickCount()
        time1_1 = (t2_1 - t1_1) / freq_1
        frame_rate_calc_1 = 1 / time1_1

    videostream_1.stop()
    cv2.destroyAllWindows()
    ser.write("ZZ".encode())


def silo():
    ser.write("SZ".encode())
    MODEL_NAME_2 = "/home/technotix/Robocon-2024/Robot 2 - Automatic/Silo Model"
    GRAPH_NAME_2 = "detect.tflite"
    LABELMAP_NAME_2 = "labelmap.txt"
    min_conf_threshold_2 = float(0.5)
    resW_2, resH_2 = 960, 540
    imW_2, imH_2 = int(resW_2), int(resH_2)
    use_TPU_2 = False

    CWD_PATH = os.getcwd()
    PATH_TO_CKPT_2 = os.path.join(CWD_PATH, MODEL_NAME_2, GRAPH_NAME_2)
    PATH_TO_LABELS_2 = os.path.join(CWD_PATH, MODEL_NAME_2, LABELMAP_NAME_2)

    with open(PATH_TO_LABELS_2, "r") as f:
        labels_2 = [line.strip() for line in f.readlines()]

    if labels_2[0] == "???":
        del labels_2[0]

    interpreter_2 = Interpreter(model_path=PATH_TO_CKPT_2)

    interpreter_2.allocate_tensors()

    resW, resH = 1280, 720
    imW, imH = int(resW), int(resH)

    input_details_2 = interpreter_2.get_input_details()
    output_details_2 = interpreter_2.get_output_details()
    height_2 = input_details_2[0]["shape"][1]
    width_2 = input_details_2[0]["shape"][2]
    mid_line_height = int(imH // 2) - 75

    floating_model_2 = input_details_2[0]["dtype"] == np.float32
    input_mean_2 = 127.5
    input_std_2 = 127.5
    outname_2 = output_details_2[0]["name"]

    if "StatefulPartitionedCall" in outname_2:
        boxes_idx_2, classes_idx_2, scores_idx_2 = 1, 3, 0
    else:
        boxes_idx_2, classes_idx_2, scores_idx_2 = 0, 1, 2

    frame_rate_calc_2 = 1
    freq_2 = cv2.getTickFrequency()

    videostream_2 = VideoStream(
        resolution=(imW_2, imH_2), framerate=30, source=0
    ).start()
    time.sleep(1)

    tracker_2 = None
    tracking_2 = False
    track_box_2 = None
    last_model_detect_2 = datetime.now() - timedelta(seconds=10)
    while True:
        t1_2 = cv2.getTickCount()
        frame_2 = videostream_2.read()
        frame_rgb_2 = cv2.cvtColor(frame_2, cv2.COLOR_BGR2RGB)
        print(ir1.get_value(), ir2.get_value())
        aligning = False

        if ir1.get_value() == 0 and ir2.get_value() == 0:
            ser.write("SZ".encode())
            while ir4.get_value() != 0:
                ser.write("TZ".encode())
            aligning = False
            return
        elif ir1.get_value() == 0:
            ser.write("RZ".encode())
            aligning = True
        elif ir2.get_value() == 0:
            ser.write("LZ".encode())
            aligning = True
        if aligning:
            continue

        if (
            ir1.get_value() == 1
            and ir2.get_value() == 1
            and tracking_2
            and last_model_detect_2 + timedelta(seconds=5) > datetime.now()
        ):
            success_2, track_box_2 = tracker_2.update(frame_2)
            if success_2:
                xmin_2, ymin_2 = track_box_2[0], track_box_2[1]
                xmax_2 = track_box_2[2] + xmin_2
                ymax_2 = track_box_2[3] + ymin_2
                p1_2 = (int(track_box_2[0]), int(track_box_2[1]))
                p2_2 = (
                    int(track_box_2[0] + track_box_2[2]),
                    int(track_box_2[1] + track_box_2[3]),
                )
                cv2.rectangle(frame_2, p1_2, p2_2, (255, 0, 0), 2, 1)

                box_center_x = (xmin_2 + xmax_2) / 2
                box_center_y = (ymin_2 + ymax_2) / 2

                if box_center_y > mid_line_height:
                    center_line_1 = int(imW // 2 - 150)
                    center_line_2 = int(imW // 2 + 150)
                else:
                    center_line_1 = int(imW // 2 - 75)
                    center_line_2 = int(imW // 2 + 75)

                if (
                    xmax_2 - xmin_2 > 0.8 * resW
                    and not box_center_x < center_line_1
                    and not box_center_x > center_line_2
                ):
                    cv2.putText(
                        frame,
                        "Stop",
                        (50, 100),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    ser.write("BZ".encode())
                    ser.write("BZ".encode())
                    ser.write("TZ".encode())
                    cv2.putText(
                        frame_2,
                        "Throw",
                        (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                elif box_center_x < center_line_1:
                    cv2.putText(
                        frame_2,
                        "Left",
                        (50, 100),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    ser.write("LZ".encode())
                elif box_center_x > center_line_2:
                    cv2.putText(
                        frame_2,
                        "Right",
                        (50, 100),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    ser.write("RZ".encode())
                elif center_line_1 < box_center_x < center_line_2:
                    cv2.putText(
                        frame_2,
                        "Forward",
                        (50, 100),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    ser.write("BZ".encode())
                    cv2.putText(
                        frame_2,
                        "Center",
                        (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                else:
                    cv2.putText(
                        frame_2,
                        "Rotate",
                        (50, 100),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    ser.write("rZ".encode())

                cv2.rectangle(
                    frame_2, (xmin_2, ymin_2), (xmax_2, ymax_2), (10, 255, 0), 2
                )
                print(classes_2[final_silo])
                object_name = labels_2[int(classes_2[final_silo])]
                label = "%s: %d%%" % (object_name, int(scores_2[final_silo] * 100))
                label_size, base_line = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2
                )
                label_ymin = max(ymin_2, label_size[1] + 10)
                cv2.rectangle(
                    frame_2,
                    (xmin_2, label_ymin - label_size[1] - 10),
                    (xmin_2 + label_size[0], label_ymin + base_line - 10),
                    (255, 255, 255),
                    cv2.FILLED,
                )
                cv2.putText(
                    frame_2,
                    label,
                    (xmin_2, label_ymin - 7),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 0),
                    2,
                )
            else:
                tracking_2 = False
                last_model_detect_2 = datetime.now() - timedelta(seconds=10)
        else:
            last_model_detect_2 = datetime.now()
            image_rgb_2 = cv2.resize(frame_rgb_2, (width_2, height_2))
            input_data_2 = np.expand_dims(image_rgb_2, axis=0)
            input_data_2 = (np.float32(input_data_2) - input_mean_2) / input_std_2
            interpreter_2.set_tensor(input_details_2[0]["index"], input_data_2)
            interpreter_2.invoke()
            boxes_2 = interpreter_2.get_tensor(output_details_2[boxes_idx_2]["index"])[
                0
            ]  # Bounding box coordinates of detected objects
            classes_2 = interpreter_2.get_tensor(
                output_details_2[classes_idx_2]["index"]
            )[
                0
            ]  # Class index of detected objects
            scores_2 = interpreter_2.get_tensor(
                output_details_2[scores_idx_2]["index"]
            )[
                0
            ]  # Confidence of detected objects

            final_silo = None
            for i in range(len(scores_2)):
                if (scores_2[i] > min_conf_threshold_2) and (scores_2[i] <= 1.0):
                    if final_silo is None or (
                        (
                            classes_2[i] == 2.0
                            and (
                                classes_2[final_silo] != 2.0
                                or scores_2[i] > scores_2[final_silo]
                            )
                        )
                        or (
                            classes_2[i] == 0.0
                            and (
                                classes_2[final_silo] != 2.0
                                and (
                                    classes_2[final_silo] != 0.0
                                    or scores_2[i] > scores_2[final_silo]
                                )
                            )
                        )
                        or (
                            classes_2[i] == 1.0
                            and (
                                classes_2[final_silo] != 2.0
                                and classes_2[final_silo] != 0.0
                                and scores_2[i] > scores_2[final_silo]
                            )
                        )
                    ):
                        final_silo = i

            if final_silo is not None:
                ymin_2, xmin_2, ymax_2, xmax_2 = (
                    int(max(1, (boxes_2[final_silo][0] * imH_2))),
                    int(max(1, (boxes_2[final_silo][1] * imW_2))),
                    int(min(imH_2, (boxes_2[final_silo][2] * imH_2))),
                    int(min(imW_2, (boxes_2[final_silo][3] * imW_2))),
                )
                p1_2 = (xmin_2, ymin_2)
                p2_2 = (xmax_2, ymax_2)
                cv2.rectangle(frame_2, p1_2, p2_2, (0, 255, 0), 2, 1)
                cv2.putText(
                    frame_2,
                    labels_2[int(classes_2[final_silo])],
                    (xmin_2, ymin_2 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 0),
                    2,
                )
                track_box_2 = (xmin_2, ymin_2, xmax_2 - xmin_2, ymax_2 - ymin_2)
                tracker_2 = cv2.TrackerKCF_create()
                tracker_2.init(frame_2, track_box_2)
                tracking_2 = True
                last_model_detect_2 = datetime.now()
            else:
                ser.write("rZ".encode())

        t2_2 = cv2.getTickCount()
        time1_2 = (t2_2 - t1_2) / freq_2
        frame_rate_calc_2 = 1 / time1_2
        cv2.putText(
            frame_2,
            "FPS: %.2f" % frame_rate_calc_2,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow("Silo Detection", frame_2)

        if cv2.waitKey(1) == ord("q"):
            break

    videostream_2.stop()
    cv2.destroyAllWindows()


def initLine():
    print("Line")
    ser.write("QZ".encode())
    while True:
        print(ser.read())
        if ser.read() != b"":
            break


initLine()
while True:
    ball()
    silo()
    print("End")
