#!/usr/bin/env python3
"""
Visualizer Node — publish 4 views gộp thành 1 grid 2x2
  /perception/debug_grid   (resize về cùng kích thước, ghép 2x2)
  /perception/view1_raw
  /perception/view2_bev
  /perception/view3_threshold
  /perception/view4_lanes
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

SRC = np.float32([[172, 151], [471, 151], [609, 368], [29, 368]])
DST = np.float32([[50,0],[250,0],[250,500],[50,500]])
BEV_W, BEV_H = 300, 500
SCALE = 0.50 / (250 - 50)
N_WINDOWS = 10
WIN_W     = 60
MIN_PIX   = 20

GRID_W = 320   # mỗi cell trong grid
GRID_H = 260

CLR_LEFT   = (255, 80,  0)
CLR_RIGHT  = (0,   80, 255)
CLR_CENTER = (0,  255, 255)
CLR_ROI    = (0,  255,  0)
CLR_TEXT   = (0,  255, 255)
CLR_REF    = (0,  255,  0)


def put_text(img, text, row, color=(255,255,255), x0=5, y0=20, dy=18):
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, text, (x0, y0+row*dy), font, 0.42, (0,0,0), 3)
    cv2.putText(img, text, (x0, y0+row*dy), font, 0.42, color,   1)


def to_cell(img):
    """Resize ảnh về GRID_W x GRID_H để ghép grid."""
    return cv2.resize(img, (GRID_W, GRID_H))


class VisualizerNode(Node):
    def __init__(self):
        super().__init__("visualizer_node")
        self.bridge = CvBridge()
        self.H = cv2.getPerspectiveTransform(SRC, DST)

        self.sub  = self.create_subscription(
            Image, "/camera/image_raw", self.callback, 10)
        self.pub1 = self.create_publisher(Image, "/perception/view1_raw",       10)
        self.pub2 = self.create_publisher(Image, "/perception/view2_bev",       10)
        self.pub3 = self.create_publisher(Image, "/perception/view3_threshold", 10)
        self.pub4 = self.create_publisher(Image, "/perception/view4_lanes",     10)
        self.pubG = self.create_publisher(Image, "/perception/debug_grid",      10)

        self.get_logger().info("Visualizer node started → /perception/debug_grid")

    def callback(self, msg):
        hdr = msg.header
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # ── VIEW 1: Raw + ROI ──
        v1 = img.copy()
        roi_pts = SRC.astype(np.int32).reshape((-1,1,2))
        overlay = v1.copy()
        cv2.fillPoly(overlay, [roi_pts], (0,255,0))
        cv2.addWeighted(overlay, 0.15, v1, 0.85, 0, v1)
        cv2.polylines(v1, [roi_pts], True, CLR_ROI, 2)
        for i,(x,y) in enumerate(SRC.astype(int)):
            cv2.circle(v1, (x,y), 5, CLR_ROI, -1)
        put_text(v1, "1: Raw + ROI", 0, (200,200,200))

        # ── VIEW 2: BEV ──
        bev = cv2.warpPerspective(img, self.H, (BEV_W, BEV_H))
        v2  = bev.copy()
        for x in range(0, BEV_W, 50):
            cv2.line(v2,(x,0),(x,BEV_H),(50,50,50),1)
        for y in range(0, BEV_H, 50):
            cv2.line(v2,(0,y),(BEV_W,y),(50,50,50),1)
        cv2.line(v2,(BEV_W//2,0),(BEV_W//2,BEV_H),CLR_REF,1)
        put_text(v2, "2: BEV", 0, (200,200,200))

        # ── Preprocessing ──
        gray   = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
        blur   = cv2.GaussianBlur(gray,(5,5),0)
        binary = cv2.adaptiveThreshold(
            blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,35,10)

        # ── VIEW 3: Threshold + histogram ──
        v3 = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        hist = np.sum(binary[BEV_H//2:,:],axis=0).astype(np.float32)
        hist_norm = (hist/(hist.max()+1e-6)*50).astype(int)
        for x,h in enumerate(hist_norm):
            cv2.line(v3,(x,BEV_H),(x,BEV_H-h),(0,180,255),1)
        cv2.line(v3,(BEV_W//2,0),(BEV_W//2,BEV_H),CLR_REF,1)
        put_text(v3,"3: Threshold",0,(200,200,200))

        # ── VIEW 4: Lanes ──
        left_pts, right_pts, boxes = self._sliding_window(binary)
        v4 = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        for (xl1,xl2,xr1,xr2,yt,yb) in boxes:
            cv2.rectangle(v4,(xl1,yt),(xl2,yb),(80,80,0),1)
            cv2.rectangle(v4,(xr1,yt),(xr2,yb),(0,80,80),1)
        for (x,y) in left_pts:
            cv2.circle(v4,(x,y),5,CLR_LEFT,-1)
        for (x,y) in right_pts:
            cv2.circle(v4,(x,y),5,CLR_RIGHT,-1)

        e_y = e_psi = kappa = None
        if len(left_pts)>=4 and len(right_pts)>=4:
            pl = np.polyfit([p[1] for p in left_pts],
                            [p[0] for p in left_pts], 3)
            pr = np.polyfit([p[1] for p in right_pts],
                            [p[0] for p in right_pts], 3)
            pc = (pl+pr)/2.0
            yr = np.linspace(0,BEV_H,200)
            pts_l=[]; pts_r=[]; pts_c=[]
            for yv in yr:
                xl=int(np.polyval(pl,yv)); xr=int(np.polyval(pr,yv))
                xc=int(np.polyval(pc,yv))
                if 0<=xl<BEV_W: pts_l.append((xl,int(yv)))
                if 0<=xr<BEV_W: pts_r.append((xr,int(yv)))
                if 0<=xc<BEV_W: pts_c.append((xc,int(yv)))
            if len(pts_l)>1: cv2.polylines(v4,[np.array(pts_l)],False,CLR_LEFT,2)
            if len(pts_r)>1: cv2.polylines(v4,[np.array(pts_r)],False,CLR_RIGHT,2)
            if len(pts_c)>1: cv2.polylines(v4,[np.array(pts_c)],False,CLR_CENTER,2)
            xc_bot = np.polyval(pc,BEV_H)
            e_y   = float((xc_bot-BEV_W/2)*SCALE)
            dxdy  = 3*pc[0]*BEV_H**2+2*pc[1]*BEV_H+pc[2]
            e_psi = float(np.arctan(dxdy*SCALE))
            kappa = float(2.0*pc[1]*SCALE**2)
            cv2.arrowedLine(v4,(BEV_W//2,BEV_H-5),(int(xc_bot),BEV_H-5),
                            (0,255,128),2,tipLength=0.3)

        cv2.line(v4,(BEV_W//2,0),(BEV_W//2,BEV_H),CLR_REF,1)
        put_text(v4,"4: Lanes",0,(200,200,200))
        if e_y is not None:
            put_text(v4,f"e_y:  {e_y:+.4f}m",   1,CLR_TEXT)
            put_text(v4,f"epsi: {np.degrees(e_psi):+.1f}d",2,CLR_TEXT)
            put_text(v4,f"kap:  {kappa:+.4f}",   3,CLR_TEXT)
        else:
            put_text(v4,"NO LANE",1,(0,0,255))

        # ── Publish individual topics ──
        for pub,view in [(self.pub1,v1),(self.pub2,v2),
                         (self.pub3,v3),(self.pub4,v4)]:
            m = self.bridge.cv2_to_imgmsg(view,"bgr8")
            m.header = hdr
            pub.publish(m)

        # ── Ghép grid 2x2 ──
        c1 = to_cell(v1); c2 = to_cell(v2)
        c3 = to_cell(v3); c4 = to_cell(v4)

        # Border mỏng phân cách
        border = 2
        top    = np.concatenate([c1,
                                  np.zeros((GRID_H,border,3),np.uint8)+60,
                                  c2], axis=1)
        bot    = np.concatenate([c3,
                                  np.zeros((GRID_H,border,3),np.uint8)+60,
                                  c4], axis=1)
        div    = np.zeros((border, GRID_W*2+border, 3), np.uint8)+60
        grid   = np.concatenate([top, div, bot], axis=0)

        # Label góc mỗi cell
        for label,(ox,oy) in [
            ("RAW+ROI", (5,        GRID_H-8)),
            ("BEV",     (GRID_W+border+5, GRID_H-8)),
            ("THRESH",  (5,        GRID_H*2+border-8)),
            ("LANES",   (GRID_W+border+5, GRID_H*2+border-8)),
        ]:
            cv2.putText(grid, label, (ox,oy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180,180,180), 1)

        gm = self.bridge.cv2_to_imgmsg(grid,"bgr8")
        gm.header = hdr
        self.pubG.publish(gm)

    def _sliding_window(self, binary):
        hist    = np.sum(binary[BEV_H//2:,:],axis=0)
        mid     = BEV_W//2
        left_x  = int(np.argmax(hist[:mid]))
        right_x = int(np.argmax(hist[mid:]))+mid
        win_h   = BEV_H//N_WINDOWS
        left_pts,right_pts,boxes = [],[],[]
        for w in range(N_WINDOWS):
            yt=BEV_H-(w+1)*win_h; yb=BEV_H-w*win_h
            xl1=max(left_x-WIN_W//2,0);   xl2=min(left_x+WIN_W//2,BEV_W)
            xr1=max(right_x-WIN_W//2,0);  xr2=min(right_x+WIN_W//2,BEV_W)
            boxes.append((xl1,xl2,xr1,xr2,yt,yb))
            rl=binary[yt:yb,xl1:xl2]; rr=binary[yt:yb,xr1:xr2]
            pl=np.where(rl>0);        pr=np.where(rr>0)
            if len(pl[1])>MIN_PIX:
                left_x=int(np.mean(pl[1]))+xl1
                left_pts.append((left_x,(yt+yb)//2))
            if len(pr[1])>MIN_PIX:
                right_x=int(np.mean(pr[1]))+xr1
                right_pts.append((right_x,(yt+yb)//2))
        return left_pts,right_pts,boxes


def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
