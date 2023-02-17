

import numpy as np

import cv2


def corner_detection(img1, img2, last_corners,i):
    """
    Detects the corners of a marker in a pair of images and returns the detected corners.

    Args:
        img1 (numpy.ndarray): First image in the pair.
        img2 (numpy.ndarray): Second image in the pair.
        last_corners (numpy.ndarray): Last detected corners.
        i (int): Iteration number.

    Returns:
        corners (numpy.ndarray): Detected corners of the marker (or an empty array if no marker was detected).
        success (bool): Indicates whether the detection was successful.
    """

    # Marker detection
    corners2 = detect_marker(img2, False, i)  # Corner order: left up, right up, right down, left down (pixel)
    corners1 = detect_marker(img1, False, i)  # Corner order: left up, right up, right down, left down (pixel)
    if corners2.shape[0] < 1 and corners1.shape[0] < 1:
        print("Detection error at iteration {}: 0 markers were detected".format(i))
        corners = last_corners
    elif corners2.shape[0] < 1:
        corners = corners1
    else:
        corners = corners2

    if corners.shape[0] > 1:
        print("Detection error at iteration {}: {} markers were detected \n".format(i, corners.shape[0]))
        if last_corners.shape[0] < 1:
            print("Last corners is an empty array at iteration {}".format(i))
            return [], False
        else:
            diff = []
            for j in range(corners.shape[0]):
                diff.append(np.linalg.norm(corners[j] - last_corners))
            index = np.argmin(diff)
            corners = corners[index]
    return np.int0(corners.reshape((4, 2))), True


def detect_marker(img ,display_marker ,iteration):
    """
    This function takes an image, detects aruco markers and returns the position of their four corners in the image in a clockwise order starting with top left..

    Parameters:
        img (numpy.array): Input image.
        display_marker (boolean): True if the visual of the detected marker should be saved.
        iteration (int): Number to specify the frame.

      Returns:
        numpy.array: The estimated positions of the aruco markers in the image.
    """

    # Returns A list containing the (x, y)-coordinates of our detected ArUco markers
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # Change image tovgrayscale
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(gray_img, arucoDict, parameters=arucoParams)

    # Display the resulting frame
    if display_marker:
        if len(corners ) >0 :
            draw_img = img.copy()
            cv2.aruco.drawDetectedMarkers(draw_img, corners)  # Draw A square around the markers
            cv2.imwrite('results/marker_image{}.png'.format(iteration), draw_img)
    return np.array(corners)

