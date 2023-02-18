import os
import cv2
import sys
import argparse

def main(args):
    videopath = args.videopath
    savepath = "./results/downSample"
    filtpath = "./results/filt"
    kernel = args.kernel

    if not os.path.exists(savepath):
        os.makedirs(savepath)
    if not os.path.exists(filtpath):
        os.makedirs(filtpath)

    v = cv2.VideoCapture(videopath)
    fps = v.get(cv2.CAP_PROP_FPS)
    w = int(v.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(v.get(cv2.CAP_PROP_FRAME_HEIGHT))

    i = 1
    cnt = 1
    flag = 1
    while True:
        ret, frame = v.read()
        if not ret:
            break
        if cnt == flag:
            frame = cv2.resize(frame, (w//2, h//2), cv2.INTER_CUBIC)
            cv2.imwrite(savepath + "/" + "%04d"%i + ".jpg", frame)
            img_gaussian = cv2.GaussianBlur(frame, (kernel, kernel), 0)
            cv2.imwrite(filtpath + "/" + "%04d"%i + ".jpg", img_gaussian)
            flag += round(fps/args.dshz)
            i += 1
        cnt += 1
    print("Done!")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Downsample video, please input the video path")
    parser.add_argument("-v", "--videopath", type=str, default = "../Video/DJI_0212.MOV", help="video path")
    parser.add_argument("-hz", "--dshz", type=int, default = 2, help="The frequency of sampling down")
    parser.add_argument("-kernel", "--kernel", type=int, default = 50, help="The kernel size of gaussian filter")
    args = parser.parse_args()
    main(args)