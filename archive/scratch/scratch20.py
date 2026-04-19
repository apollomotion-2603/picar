import numpy as np
import cv2

def identify_lanes(histogram, expected_width=200):
    # smooth histogram
    hist_smooth = np.convolve(histogram, np.ones(10)/10, mode='same')
    
    # find local maxima
    maxima = []
    for i in range(10, len(hist_smooth)-10):
        if hist_smooth[i] > 10 and \
           hist_smooth[i] == np.max(hist_smooth[i-10:i+11]):
            maxima.append(i)
            
    # sort by prominence/height
    maxima.sort(key=lambda x: hist_smooth[x], reverse=True)
    
    if len(maxima) == 0:
        return 100, 300
        
    p1 = maxima[0]
    
    if len(maxima) > 1:
        # Find another peak that is roughly expected_width away
        for p2 in maxima[1:]:
            if 150 <= abs(p1 - p2) <= 250:
                left_base = min(p1, p2)
                right_base = max(p1, p2)
                return left_base, right_base

    # Only one valid peak found. Is it left or right?
    # If it's left, the right lane should be at p1 + expected_width.
    # If p1 + expected_width is outside the image (>400), then p1 MUST be the right lane!
    # Wait, if p1 is left, and p1 + expected_width > 400, then right lane is outside!
    # So p1 must be the left lane. wait.
    # If p1=250. p1 + 200 = 450 (outside). So p1 is Left lane? No! If p1=250, and it's left lane, right is outside.
    # But if p1=250, and it's right lane, left is at 50! So left SHOULD be visible!
    # But there's no peak at 50! So p1 must be the LEFT lane?
    # Wait, if left SHOULD be visible at 50, but we didn't find it... then maybe p1 is actually the RIGHT lane and left was obscured?
    # Let's use simple logic: default left is 100, right is 300.
    if p1 < 200:
        return p1, p1 + expected_width
    else:
        return p1 - expected_width, p1

print(identify_lanes(np.zeros(400)))
