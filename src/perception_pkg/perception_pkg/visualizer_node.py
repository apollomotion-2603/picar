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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from lane_msgs.msg import LaneState
import cv2
import numpy as np

# Constants chỉ dùng cho grid layout và màu sắc — không liên quan đến BEV

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

        # Đọc params từ yaml (giống perception_node — phải khới động cùng params-file)
        self.declare_parameter('bev_src', [172.0,151.0,471.0,151.0,609.0,368.0,29.0,368.0])
        self.declare_parameter('bev_dst', [100.0,0.0,300.0,0.0,300.0,500.0,100.0,500.0])
        self.declare_parameter('bev_width',  400)
        self.declare_parameter('bev_height', 500)
        self.declare_parameter('bev_scale',  0.0025)
        self.declare_parameter('n_windows',  10)
        self.declare_parameter('win_width',  80)
        self.declare_parameter('min_pixels', 15)

        src_flat = self.get_parameter('bev_src').value
        dst_flat = self.get_parameter('bev_dst').value
        self.BEV_W     = self.get_parameter('bev_width').value
        self.BEV_H     = self.get_parameter('bev_height').value
        self.SCALE     = self.get_parameter('bev_scale').value
        self.N_WIN     = self.get_parameter('n_windows').value
        self.WIN_W     = self.get_parameter('win_width').value
        self.MIN_PIX   = self.get_parameter('min_pixels').value

        SRC = np.float32(src_flat).reshape(4, 2)
        DST = np.float32(dst_flat).reshape(4, 2)
        self.SRC = SRC
        self.H   = cv2.getPerspectiveTransform(SRC, DST)
        self.get_logger().info(
            f'Visualizer: BEV={self.BEV_W}x{self.BEV_H} scale={self.SCALE}m/px '
            f'WIN_W={self.WIN_W}px')

        # QoS: depth=1 → chỉ xem frame mới nhất, không để frame cũ tích lũ (giảm lag)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sub  = self.create_subscription(
            Image, "/camera/image_raw", self.callback, sensor_qos)
        self.pub1 = self.create_publisher(Image, "/perception/view1_raw",       1)
        self.pub2 = self.create_publisher(Image, "/perception/view2_bev",       1)
        self.pub3 = self.create_publisher(Image, "/perception/view3_threshold", 1)
        self.pub4 = self.create_publisher(Image, "/perception/view4_lanes",     1)
        self.pubG = self.create_publisher(Image, "/perception/debug_grid",      1)

        self.get_logger().info("Visualizer node started → /perception/debug_grid (queue=1, best-effort)")

        # Sub lane_state — lấy giá trị thực tế từ perception_node (arc-length fit)
        self.lane_state = None
        self.create_subscription(
            LaneState, '/perception/lane_state', self._lane_state_cb, 10)


    def callback(self, msg):
        hdr = msg.header
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # ── VIEW 1: Raw + ROI ──
        v1 = img.copy()
        roi_pts = self.SRC.astype(np.int32).reshape((-1,1,2))
        overlay = v1.copy()
        cv2.fillPoly(overlay, [roi_pts], (0,255,0))
        cv2.addWeighted(overlay, 0.15, v1, 0.85, 0, v1)
        cv2.polylines(v1, [roi_pts], True, CLR_ROI, 2)
        for i,(x,y) in enumerate(self.SRC.astype(int)):
            cv2.circle(v1, (x,y), 5, CLR_ROI, -1)
        put_text(v1, "1: Raw + ROI", 0, (200,200,200))

        # ── VIEW 2: BEV ──
        bev = cv2.warpPerspective(img, self.H, (self.BEV_W, self.BEV_H))
        v2  = bev.copy()
        for x in range(0, self.BEV_W, 50):
            cv2.line(v2,(x,0),(x,self.BEV_H),(50,50,50),1)
        for y in range(0, self.BEV_H, 50):
            cv2.line(v2,(0,y),(self.BEV_W,y),(50,50,50),1)
        cv2.line(v2,(self.BEV_W//2,0),(self.BEV_W//2,self.BEV_H),CLR_REF,1)
        put_text(v2, "2: BEV", 0, (200,200,200))

        # ── Preprocessing ──
        gray   = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
        blur   = cv2.GaussianBlur(gray,(5,5),0)
        binary = cv2.adaptiveThreshold(
            blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,35,10)

        # ── VIEW 3: Threshold + histogram ──
        v3 = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        hist = np.sum(binary[self.BEV_H//2:,:],axis=0).astype(np.float32)
        hist_norm = (hist/(hist.max()+1e-6)*50).astype(int)
        for x,h in enumerate(hist_norm):
            cv2.line(v3,(x,self.BEV_H),(x,self.BEV_H-h),(0,180,255),1)
        cv2.line(v3,(self.BEV_W//2,0),(self.BEV_W//2,self.BEV_H),CLR_REF,1)
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
            yr = np.linspace(0,self.BEV_H,200)
            pts_l=[]; pts_r=[]; pts_c=[]
            for yv in yr:
                xl=int(np.polyval(pl,yv)); xr=int(np.polyval(pr,yv))
                xc=int(np.polyval(pc,yv))
                if 0<=xl<self.BEV_W: pts_l.append((xl,int(yv)))
                if 0<=xr<self.BEV_W: pts_r.append((xr,int(yv)))
                if 0<=xc<self.BEV_W: pts_c.append((xc,int(yv)))
            if len(pts_l)>1: cv2.polylines(v4,[np.array(pts_l)],False,CLR_LEFT,2)
            if len(pts_r)>1: cv2.polylines(v4,[np.array(pts_r)],False,CLR_RIGHT,2)
            if len(pts_c)>1: cv2.polylines(v4,[np.array(pts_c)],False,CLR_CENTER,2)
            xc_bot = np.polyval(pc,self.BEV_H)
            e_y   = float((xc_bot-self.BEV_W/2)*self.SCALE)
            dxdy  = 3*pc[0]*self.BEV_H**2+2*pc[1]*self.BEV_H+pc[2]
            e_psi = float(np.arctan(dxdy*self.SCALE))
            kappa = float(2.0*pc[1]*self.SCALE**2)
            cv2.arrowedLine(v4,(self.BEV_W//2,self.BEV_H-5),(int(xc_bot),self.BEV_H-5),
                            (0,255,128),2,tipLength=0.3)

        cv2.line(v4,(self.BEV_W//2,0),(self.BEV_W//2,self.BEV_H),CLR_REF,1)
        put_text(v4,"4: Lanes",0,(200,200,200))

        # ── Overlay giá trị THỰC TỎa từ perception_node (arc-length fit) ──
        ls = self.lane_state
        if ls is not None and ls.lane_detected:
            # Vẽ đường center từ hệ số polynomial thực tế (coeff_a..d)
            # x_c(s) = a*s^3 + b*s^2 + c*s + d, với s đo bằng [m]
            # Convert s → pixel y: s=0 → y=BEV_H (bottom), s thăng theo y giảm
            pts_real = []
            for sv in np.linspace(0.0, min(ls.s_max, 0.65), 40):
                xc_m = ls.coeff_a*sv**3 + ls.coeff_b*sv**2 + ls.coeff_c*sv + ls.coeff_d
                xc_px = int(xc_m/self.SCALE + self.BEV_W/2)
                yc_px = int(self.BEV_H - sv/self.SCALE * self.SCALE / self.SCALE
                            * (self.BEV_H / (ls.s_max if ls.s_max > 0.05 else 0.7)))
                yc_px = int(self.BEV_H * (1.0 - sv / (ls.s_max if ls.s_max > 0.05 else 0.7)))
                if 0 <= xc_px < self.BEV_W and 0 <= yc_px < self.BEV_H:
                    pts_real.append((xc_px, yc_px))
            if len(pts_real) > 1:
                cv2.polylines(v4, [np.array(pts_real)], False, (0, 255, 0), 3)  # GREEN = real

            put_text(v4, f"e_y : {ls.e_y*1000:+.1f}mm",     1, (0, 255, 0))
            put_text(v4, f"epsi: {np.degrees(ls.e_psi):+.1f}d", 2, (0, 255, 0))
            put_text(v4, f"kap : {ls.kappa:+.4f}",           3, (0, 255, 0))
        elif ls is not None and not ls.lane_detected:
            put_text(v4, "LOST", 1, (0, 0, 255))
        else:
            put_text(v4, "wait lane_state", 1, (100, 100, 100))
        # Giữ lại text approximate nếu không có lane_state
        if ls is None and e_y is not None:
            put_text(v4, f"~e_y:  {e_y:+.4f}m",          1, CLR_TEXT)
            put_text(v4, f"~epsi: {np.degrees(e_psi):+.1f}d", 2, CLR_TEXT)
            put_text(v4, f"~kap:  {kappa:+.4f}",          3, CLR_TEXT)

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
        hist    = np.sum(binary[self.BEV_H//2:,:],axis=0)
        mid     = self.BEV_W//2
        left_x  = int(np.argmax(hist[:mid]))
        right_x = int(np.argmax(hist[mid:]))+mid
        win_h   = self.BEV_H//self.N_WIN
        left_pts,right_pts,boxes = [],[],[]
        for w in range(self.N_WIN):
            yt=self.BEV_H-(w+1)*win_h; yb=self.BEV_H-w*win_h
            xl1=max(left_x-self.WIN_W//2,0);   xl2=min(left_x+self.WIN_W//2,self.BEV_W)
            xr1=max(right_x-self.WIN_W//2,0);  xr2=min(right_x+self.WIN_W//2,self.BEV_W)
            boxes.append((xl1,xl2,xr1,xr2,yt,yb))
            rl=binary[yt:yb,xl1:xl2]; rr=binary[yt:yb,xr1:xr2]
            pl=np.where(rl>0);        pr=np.where(rr>0)
            if len(pl[1])>self.MIN_PIX:
                left_x=int(np.mean(pl[1]))+xl1
                left_pts.append((left_x,(yt+yb)//2))
            if len(pr[1])>self.MIN_PIX:
                right_x=int(np.mean(pr[1]))+xr1
                right_pts.append((right_x,(yt+yb)//2))
        return left_pts,right_pts,boxes

    def _lane_state_cb(self, msg: LaneState):
        """Cache lane state mới nhất từ perception_node để overlay lên LANES panel."""
        self.lane_state = msg



def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
