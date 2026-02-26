#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include "ASICamera2.h"
#include <opencv2/opencv.hpp>

static volatile bool running = true;
void sigHandler(int sig) { running = false; }

struct ROI {
    cv::Rect rect;
    int rotation;
    bool mirrorH;
    bool mirrorV;
    cv::Scalar color;
};

cv::Mat applyTransform(const cv::Mat& src, int rotation, bool mirrorH, bool mirrorV) {
    cv::Mat result = src.clone();
    if (mirrorH && mirrorV) cv::flip(result, result, -1);
    else if (mirrorH) cv::flip(result, result, 1);
    else if (mirrorV) cv::flip(result, result, 0);
    
    switch (rotation) {
        case 90:  cv::rotate(result, result, cv::ROTATE_90_CLOCKWISE); break;
        case 180: cv::rotate(result, result, cv::ROTATE_180); break;
        case 270: cv::rotate(result, result, cv::ROTATE_90_COUNTERCLOCKWISE); break;
    }
    return result;
}

cv::Rect clampRect(cv::Rect r, int maxW, int maxH) {
    r.x = std::max(0, std::min(r.x, maxW - 50));
    r.y = std::max(0, std::min(r.y, maxH - 50));
    r.width = std::min(r.width, maxW - r.x);
    r.height = std::min(r.height, maxH - r.y);
    r.width = std::max(50, r.width);
    r.height = std::max(50, r.height);
    return r;
}

// Global state
int activeROI = 0;
bool dragging = false;
bool resizing = false;
cv::Point dragStart;
cv::Rect dragOrigRect;
ROI rois[2];
long exposure = 50000;
int gain = 80;
int saveCount = 0;
int camID = 0;
int frameWidth = 1920;
int frameHeight = 1080;
int displayW = 960;
int displayH = 540;
bool autoExposure = false;
int magnification = 1;  // 1x, 2x, 3x, 5x, 8x
const int magLevels[] = {1, 2, 3, 5, 8};
int magIndex = 0;
int roiBaseSize = 400;  // base ROI size in sensor pixels

// Button actions
enum Actions {
    ACT_SEL_ROI1 = 0, ACT_SEL_ROI2,
    ACT_ROT_CW, ACT_ROT_CCW,
    ACT_MIRROR_H, ACT_MIRROR_V,
    ACT_EXP_UP, ACT_EXP_DOWN,
    ACT_GAIN_UP, ACT_GAIN_DOWN,
    ACT_SAVE, ACT_RESET,
    ACT_AUTO_EXP, ACT_MAG_UP, ACT_MAG_DOWN
};

void handleAction(int action) {
    switch (action) {
        case ACT_SEL_ROI1: activeROI = 0; break;
        case ACT_SEL_ROI2: activeROI = 1; break;
        case ACT_ROT_CW:
            rois[activeROI].rotation = (rois[activeROI].rotation + 90) % 360;
            break;
        case ACT_ROT_CCW:
            rois[activeROI].rotation = (rois[activeROI].rotation + 270) % 360;
            break;
        case ACT_MIRROR_H:
            rois[activeROI].mirrorH = !rois[activeROI].mirrorH;
            break;
        case ACT_MIRROR_V:
            rois[activeROI].mirrorV = !rois[activeROI].mirrorV;
            break;
        case ACT_EXP_UP:
            exposure = std::min(exposure * 2, (long)10000000);
            ASISetControlValue(camID, ASI_EXPOSURE, exposure, ASI_FALSE);
            break;
        case ACT_EXP_DOWN:
            exposure = std::max(exposure / 2, (long)100);
            ASISetControlValue(camID, ASI_EXPOSURE, exposure, ASI_FALSE);
            break;
        case ACT_GAIN_UP:
            gain = std::min(gain + 10, 300);
            ASISetControlValue(camID, ASI_GAIN, gain, ASI_FALSE);
            break;
        case ACT_GAIN_DOWN:
            gain = std::max(gain - 10, 0);
            ASISetControlValue(camID, ASI_GAIN, gain, ASI_FALSE);
            break;
        case ACT_SAVE:
            saveCount++;
            break;
        case ACT_RESET:
            rois[activeROI].rotation = 0;
            rois[activeROI].mirrorH = false;
            rois[activeROI].mirrorV = false;
            break;
        case ACT_AUTO_EXP:
            autoExposure = !autoExposure;
            break;
        case ACT_MAG_UP:
            if (magIndex < 4) {
                magIndex++;
                magnification = magLevels[magIndex];
                // Resize both ROIs to maintain consistent pixel mapping
                int newSize = roiBaseSize / magnification;
                newSize = std::max(50, newSize);
                for (int i = 0; i < 2; i++) {
                    int cx = rois[i].rect.x + rois[i].rect.width / 2;
                    int cy = rois[i].rect.y + rois[i].rect.height / 2;
                    rois[i].rect.width = newSize;
                    rois[i].rect.height = newSize;
                    rois[i].rect.x = cx - newSize / 2;
                    rois[i].rect.y = cy - newSize / 2;
                }
            }
            break;
        case ACT_MAG_DOWN:
            if (magIndex > 0) {
                magIndex--;
                magnification = magLevels[magIndex];
                int newSize = roiBaseSize / magnification;
                newSize = std::max(50, newSize);
                for (int i = 0; i < 2; i++) {
                    int cx = rois[i].rect.x + rois[i].rect.width / 2;
                    int cy = rois[i].rect.y + rois[i].rect.height / 2;
                    rois[i].rect.width = newSize;
                    rois[i].rect.height = newSize;
                    rois[i].rect.x = cx - newSize / 2;
                    rois[i].rect.y = cy - newSize / 2;
                }
            }
            break;
    }
}

// Auto-exposure: analyze both ROI regions and adjust exposure
void updateAutoExposure(const cv::Mat& frame) {
    if (!autoExposure) return;
    
    double totalBrightness = 0;
    int totalPixels = 0;
    
    for (int i = 0; i < 2; i++) {
        cv::Mat roi = frame(rois[i].rect);
        cv::Mat gray;
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        cv::Scalar mean = cv::mean(gray);
        totalBrightness += mean[0] * roi.total();
        totalPixels += roi.total();
    }
    
    double avgBrightness = totalBrightness / totalPixels;
    int target = 128;  // target brightness (middle gray)
    
    // Adjust exposure proportionally, with damping
    if (avgBrightness < target * 0.85 && exposure < 10000000) {
        exposure = std::min((long)(exposure * 1.15), (long)10000000);
        ASISetControlValue(camID, ASI_EXPOSURE, exposure, ASI_FALSE);
    } else if (avgBrightness > target * 1.15 && exposure > 100) {
        exposure = std::max((long)(exposure * 0.85), (long)100);
        ASISetControlValue(camID, ASI_EXPOSURE, exposure, ASI_FALSE);
    }
}

// Control panel button layout
const int PANEL_W = 250;
const int PANEL_H = 640;
const int BTN_H = 36;
const int BTN_W = 110;
const int BTN_PAD = 6;

struct PanelButton {
    cv::Rect rect;
    int action;
};
std::vector<PanelButton> panelButtons;

void drawButton(cv::Mat& img, cv::Rect r, const char* label, bool active, bool toggled) {
    cv::Scalar bg, text, border;
    if (toggled) {
        bg = cv::Scalar(50, 160, 50);
        text = cv::Scalar(255, 255, 255);
        border = cv::Scalar(80, 200, 80);
    } else if (active) {
        bg = cv::Scalar(180, 120, 40);
        text = cv::Scalar(255, 255, 255);
        border = cv::Scalar(220, 160, 60);
    } else {
        bg = cv::Scalar(60, 60, 60);
        text = cv::Scalar(200, 200, 200);
        border = cv::Scalar(100, 100, 100);
    }
    cv::rectangle(img, r, bg, -1);
    cv::rectangle(img, r, border, 1);
    int baseline;
    cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.45, 1, &baseline);
    cv::putText(img, label, cv::Point(r.x + (r.width - ts.width) / 2, r.y + (r.height + ts.height) / 2),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, text, 1, cv::LINE_AA);
}

cv::Mat drawControlPanel() {
    cv::Mat panel(PANEL_H, PANEL_W, CV_8UC3, cv::Scalar(35, 35, 35));
    panelButtons.clear();
    int y = 10;
    
    // Title
    cv::putText(panel, "DUAL ROI CONTROLS", cv::Point(20, y + 16),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    y += 32;
    cv::line(panel, cv::Point(10, y), cv::Point(PANEL_W - 10, y), cv::Scalar(80, 80, 80), 1);
    y += 12;
    
    // SELECT ROI
    cv::putText(panel, "SELECT ROI", cv::Point(10, y + 13),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
    y += 22;
    cv::Rect r1(BTN_PAD, y, BTN_W, BTN_H);
    cv::Rect r2(BTN_W + BTN_PAD * 2, y, BTN_W, BTN_H);
    drawButton(panel, r1, "ROI 1", activeROI == 0, false);
    drawButton(panel, r2, "ROI 2", activeROI == 1, false);
    panelButtons.push_back({r1, ACT_SEL_ROI1});
    panelButtons.push_back({r2, ACT_SEL_ROI2});
    y += BTN_H + 14;
    
    cv::line(panel, cv::Point(10, y), cv::Point(PANEL_W - 10, y), cv::Scalar(80, 80, 80), 1);
    y += 12;
    
    // ROTATION
    char rotLabel[32];
    sprintf(rotLabel, "ROTATION (%d deg)", rois[activeROI].rotation);
    cv::putText(panel, rotLabel, cv::Point(10, y + 13),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
    y += 22;
    cv::Rect rCW(BTN_PAD, y, BTN_W, BTN_H);
    cv::Rect rCCW(BTN_W + BTN_PAD * 2, y, BTN_W, BTN_H);
    drawButton(panel, rCW, "Rotate CW", false, false);
    drawButton(panel, rCCW, "Rotate CCW", false, false);
    panelButtons.push_back({rCW, ACT_ROT_CW});
    panelButtons.push_back({rCCW, ACT_ROT_CCW});
    y += BTN_H + 14;
    
    cv::line(panel, cv::Point(10, y), cv::Point(PANEL_W - 10, y), cv::Scalar(80, 80, 80), 1);
    y += 12;
    
    // MIRROR
    cv::putText(panel, "MIRROR", cv::Point(10, y + 13),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
    y += 22;
    cv::Rect rMH(BTN_PAD, y, BTN_W, BTN_H);
    cv::Rect rMV(BTN_W + BTN_PAD * 2, y, BTN_W, BTN_H);
    drawButton(panel, rMH, "Mirror H", false, rois[activeROI].mirrorH);
    drawButton(panel, rMV, "Mirror V", false, rois[activeROI].mirrorV);
    panelButtons.push_back({rMH, ACT_MIRROR_H});
    panelButtons.push_back({rMV, ACT_MIRROR_V});
    y += BTN_H + 14;
    
    cv::line(panel, cv::Point(10, y), cv::Point(PANEL_W - 10, y), cv::Scalar(80, 80, 80), 1);
    y += 12;
    
    // EXPOSURE
    char expLabel[64];
    sprintf(expLabel, "EXPOSURE (%.1f ms)", exposure / 1000.0);
    cv::putText(panel, expLabel, cv::Point(10, y + 13),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
    y += 22;
    cv::Rect rEU(BTN_PAD, y, BTN_W, BTN_H);
    cv::Rect rED(BTN_W + BTN_PAD * 2, y, BTN_W, BTN_H);
    drawButton(panel, rEU, "Exp +", false, false);
    drawButton(panel, rED, "Exp -", false, false);
    panelButtons.push_back({rEU, ACT_EXP_UP});
    panelButtons.push_back({rED, ACT_EXP_DOWN});
    y += BTN_H + 14;
    
    // GAIN
    char gainLabel[32];
    sprintf(gainLabel, "GAIN (%d)", gain);
    cv::putText(panel, gainLabel, cv::Point(10, y + 13),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
    y += 22;
    cv::Rect rGU(BTN_PAD, y, BTN_W, BTN_H);
    cv::Rect rGD(BTN_W + BTN_PAD * 2, y, BTN_W, BTN_H);
    drawButton(panel, rGU, "Gain +", false, false);
    drawButton(panel, rGD, "Gain -", false, false);
    panelButtons.push_back({rGU, ACT_GAIN_UP});
    panelButtons.push_back({rGD, ACT_GAIN_DOWN});
    y += BTN_H + 14;
    
    cv::line(panel, cv::Point(10, y), cv::Point(PANEL_W - 10, y), cv::Scalar(80, 80, 80), 1);
    y += 12;
    
    // AUTO EXPOSURE
    cv::putText(panel, "AUTO EXPOSURE", cv::Point(10, y + 13),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
    y += 22;
    cv::Rect rAE(BTN_PAD, y, PANEL_W - BTN_PAD * 2, BTN_H);
    drawButton(panel, rAE, autoExposure ? "Auto Exp: ON" : "Auto Exp: OFF", false, autoExposure);
    panelButtons.push_back({rAE, ACT_AUTO_EXP});
    y += BTN_H + 14;
    
    cv::line(panel, cv::Point(10, y), cv::Point(PANEL_W - 10, y), cv::Scalar(80, 80, 80), 1);
    y += 12;
    
    // MAGNIFICATION
    char magLabel[64];
    sprintf(magLabel, "MAGNIFICATION (%dx)", magnification);
    cv::putText(panel, magLabel, cv::Point(10, y + 13),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
    y += 22;
    cv::Rect rMU(BTN_PAD, y, BTN_W, BTN_H);
    cv::Rect rMD(BTN_W + BTN_PAD * 2, y, BTN_W, BTN_H);
    drawButton(panel, rMU, "Mag +", false, false);
    drawButton(panel, rMD, "Mag -", false, false);
    panelButtons.push_back({rMU, ACT_MAG_UP});
    panelButtons.push_back({rMD, ACT_MAG_DOWN});
    y += BTN_H + 14;
    
    cv::line(panel, cv::Point(10, y), cv::Point(PANEL_W - 10, y), cv::Scalar(80, 80, 80), 1);
    y += 12;
    
    // SAVE / RESET
    cv::Rect rSave(BTN_PAD, y, BTN_W, BTN_H);
    cv::Rect rReset(BTN_W + BTN_PAD * 2, y, BTN_W, BTN_H);
    drawButton(panel, rSave, "Save", false, false);
    drawButton(panel, rReset, "Reset", false, false);
    panelButtons.push_back({rSave, ACT_SAVE});
    panelButtons.push_back({rReset, ACT_RESET});
    y += BTN_H + 14;
    
    cv::line(panel, cv::Point(10, y), cv::Point(PANEL_W - 10, y), cv::Scalar(80, 80, 80), 1);
    y += 14;
    
    // ROI info
    for (int i = 0; i < 2; i++) {
        cv::Scalar col = (i == activeROI) ? cv::Scalar(0, 255, 255) : cv::Scalar(120, 120, 120);
        char info[128];
        sprintf(info, "ROI%d: %dx%d R:%d H:%s V:%s", i + 1,
                rois[i].rect.width, rois[i].rect.height,
                rois[i].rotation,
                rois[i].mirrorH ? "Y" : "N",
                rois[i].mirrorV ? "Y" : "N");
        cv::putText(panel, info, cv::Point(10, y + 12),
                    cv::FONT_HERSHEY_SIMPLEX, 0.38, col, 1, cv::LINE_AA);
        y += 20;
    }
    
    return panel;
}

// Mouse callback for main view
void onMainMouse(int event, int x, int y, int flags, void* userdata) {
    float scaleX = (float)frameWidth / displayW;
    float scaleY = (float)frameHeight / displayH;
    int fx = (int)(x * scaleX);
    int fy = (int)(y * scaleY);
    
    if (event == cv::EVENT_LBUTTONDOWN) {
        cv::Rect& r = rois[activeROI].rect;
        int cx = r.x + r.width;
        int cy = r.y + r.height;
        
        if (abs(fx - cx) < 30 && abs(fy - cy) < 30) {
            resizing = true;
            dragging = false;
            dragStart = cv::Point(fx, fy);
            dragOrigRect = r;
        } else if (r.contains(cv::Point(fx, fy))) {
            dragging = true;
            resizing = false;
            dragStart = cv::Point(fx, fy);
            dragOrigRect = r;
        }
    }
    else if (event == cv::EVENT_MOUSEMOVE) {
        if (dragging) {
            rois[activeROI].rect.x = dragOrigRect.x + (fx - dragStart.x);
            rois[activeROI].rect.y = dragOrigRect.y + (fy - dragStart.y);
        } else if (resizing) {
            // Keep square and sync both ROIs to same size
            int delta = std::max(fx - dragStart.x, fy - dragStart.y);
            int newSize = std::max(50, dragOrigRect.width + delta);
            rois[activeROI].rect.width = newSize;
            rois[activeROI].rect.height = newSize;
            // Sync the other ROI's size (keep its center position)
            int other = 1 - activeROI;
            int ocx = rois[other].rect.x + rois[other].rect.width / 2;
            int ocy = rois[other].rect.y + rois[other].rect.height / 2;
            rois[other].rect.width = newSize;
            rois[other].rect.height = newSize;
            rois[other].rect.x = ocx - newSize / 2;
            rois[other].rect.y = ocy - newSize / 2;
        }
    }
    else if (event == cv::EVENT_LBUTTONUP) {
        dragging = false;
        resizing = false;
    }
}

// Mouse callback for control panel
void onPanelMouse(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        for (auto& btn : panelButtons) {
            if (btn.rect.contains(cv::Point(x, y))) {
                handleAction(btn.action);
                break;
            }
        }
    }
}

int main() {
    signal(SIGINT, sigHandler);
    signal(SIGTERM, sigHandler);

    int numCameras = ASIGetNumOfConnectedCameras();
    if (numCameras <= 0) { printf("No cameras found.\n"); return 1; }

    ASI_CAMERA_INFO CamInfo;
    ASIGetCameraProperty(&CamInfo, 0);
    camID = CamInfo.CameraID;
    frameWidth = CamInfo.MaxWidth;
    frameHeight = CamInfo.MaxHeight;
    printf("Camera: %s (%dx%d)\n", CamInfo.Name, frameWidth, frameHeight);

    if (ASIOpenCamera(camID) != ASI_SUCCESS) { printf("Failed to open.\n"); return 1; }
    if (ASIInitCamera(camID) != ASI_SUCCESS) { ASICloseCamera(camID); return 1; }

    ASISetControlValue(camID, ASI_EXPOSURE, exposure, ASI_FALSE);
    ASISetControlValue(camID, ASI_GAIN, gain, ASI_FALSE);
    ASISetControlValue(camID, ASI_WB_R, 52, ASI_FALSE);
    ASISetControlValue(camID, ASI_WB_B, 95, ASI_FALSE);
    ASISetROIFormat(camID, frameWidth, frameHeight, 1, ASI_IMG_RGB24);

    long imgSize = (long)frameWidth * frameHeight * 3;
    unsigned char* imgBuf = (unsigned char*)malloc(imgSize);

    displayW = frameWidth / 2;
    displayH = frameHeight / 2;

    // Initialize ROIs — both same size for consistent comparison
    roiBaseSize = std::min(frameWidth / 3, frameHeight / 3);
    int initSize = roiBaseSize / magnification;
    rois[0] = {cv::Rect(100, 100, initSize, initSize), 0, false, false, cv::Scalar(0, 255, 0)};
    rois[1] = {cv::Rect(frameWidth / 2, 100, initSize, initSize), 0, false, false, cv::Scalar(0, 165, 255)};

    // Create windows
    cv::namedWindow("Main View", cv::WINDOW_NORMAL);
    cv::resizeWindow("Main View", displayW, displayH);
    cv::setMouseCallback("Main View", onMainMouse, nullptr);
    
    cv::namedWindow("Controls", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Controls", onPanelMouse, nullptr);
    
    cv::namedWindow("ROI 1", cv::WINDOW_NORMAL);
    cv::resizeWindow("ROI 1", 400, 400);
    cv::namedWindow("ROI 2", cv::WINDOW_NORMAL);
    cv::resizeWindow("ROI 2", 400, 400);

    ASIStartVideoCapture(camID);
    printf("Dual ROI Viewer started. Press Q to quit.\n");

    int frameCount = 0;
    bool needsSave = false;
    int lastSaveCount = 0;

    while (running) {
        ASI_ERROR_CODE err = ASIGetVideoData(camID, imgBuf, imgSize, 500);
        if (err != ASI_SUCCESS) continue;

        cv::Mat frame(frameHeight, frameWidth, CV_8UC3, imgBuf);
        frameCount++;

        for (int i = 0; i < 2; i++)
            rois[i].rect = clampRect(rois[i].rect, frameWidth, frameHeight);

        // Auto-exposure update (every 10 frames to reduce overhead)
        if (autoExposure && frameCount % 10 == 0)
            updateAutoExposure(frame);

        // Main view (scaled)
        cv::Mat display;
        cv::resize(frame, display, cv::Size(displayW, displayH));
        
        float sx = (float)displayW / frameWidth;
        float sy = (float)displayH / frameHeight;
        
        for (int i = 0; i < 2; i++) {
            cv::Rect scaled((int)(rois[i].rect.x * sx), (int)(rois[i].rect.y * sy),
                           (int)(rois[i].rect.width * sx), (int)(rois[i].rect.height * sy));
            int thickness = (i == activeROI) ? 2 : 1;
            cv::rectangle(display, scaled, rois[i].color, thickness);
            
            char label[16];
            sprintf(label, "ROI %d", i + 1);
            cv::putText(display, label, cv::Point(scaled.x + 4, scaled.y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, rois[i].color, 1, cv::LINE_AA);
            
            cv::Point br(scaled.x + scaled.width, scaled.y + scaled.height);
            cv::rectangle(display, cv::Point(br.x - 8, br.y - 8), br, rois[i].color, -1);
        }
        
        cv::imshow("Main View", display);

        // Control panel
        cv::Mat panel = drawControlPanel();
        cv::imshow("Controls", panel);

        // ROI windows — display at consistent pixel ratio
        for (int i = 0; i < 2; i++) {
            cv::Mat roiCrop = frame(rois[i].rect).clone();
            cv::Mat transformed = applyTransform(roiCrop, rois[i].rotation, rois[i].mirrorH, rois[i].mirrorV);
            
            // Scale to maintain consistent pixel mapping
            // At 1x mag, ROI pixels display 1:1
            // At 5x mag, each sensor pixel shows as 5x5 screen pixels
            int dispW = transformed.cols * magnification;
            int dispH = transformed.rows * magnification;
            // Cap display size to something reasonable
            int maxDisp = 800;
            if (dispW > maxDisp || dispH > maxDisp) {
                double scale = (double)maxDisp / std::max(dispW, dispH);
                dispW = (int)(dispW * scale);
                dispH = (int)(dispH * scale);
            }
            
            cv::Mat displayed;
            cv::resize(transformed, displayed, cv::Size(dispW, dispH), 0, 0, cv::INTER_NEAREST);
            
            char winName[16];
            sprintf(winName, "ROI %d", i + 1);
            cv::imshow(winName, displayed);
        }

        // Save
        if (saveCount != lastSaveCount) {
            for (int i = 0; i < 2; i++) {
                cv::Mat roiCrop = frame(rois[i].rect).clone();
                cv::Mat transformed = applyTransform(roiCrop, rois[i].rotation, rois[i].mirrorH, rois[i].mirrorV);
                char fname[64];
                sprintf(fname, "roi%d_save_%04d.png", i + 1, saveCount);
                cv::imwrite(fname, transformed);
                printf("Saved %s\n", fname);
            }
            lastSaveCount = saveCount;
        }

        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) break;
        switch (key) {
            case '1': activeROI = 0; break;
            case '2': activeROI = 1; break;
            case 'r': case 'R': handleAction(ACT_ROT_CW); break;
            case 'h': case 'H': handleAction(ACT_MIRROR_H); break;
            case 'v': case 'V': handleAction(ACT_MIRROR_V); break;
            case 's': case 'S': handleAction(ACT_SAVE); break;
            case '+': case '=': handleAction(ACT_EXP_UP); break;
            case '-': case '_': handleAction(ACT_EXP_DOWN); break;
            case 'a': case 'A': handleAction(ACT_AUTO_EXP); break;
            case ']': handleAction(ACT_MAG_UP); break;
            case '[': handleAction(ACT_MAG_DOWN); break;
        }
    }

    ASIStopVideoCapture(camID);
    ASICloseCamera(camID);
    free(imgBuf);
    cv::destroyAllWindows();
    printf("Done. (%d frames)\n", frameCount);
    return 0;
}
