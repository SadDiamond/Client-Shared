"""Drawing for the Radar challenge: a 2D top-down fan, a rotatable 3D ping
cloud, and grouping readings into rough "objects".

Coordinates are in cm, relative to the robot: x = right, y = forward,
z = up, with the floor at z = 0."""
import math
import time

from PyQt5.QtCore import Qt, QPointF, QRectF, QTimer
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QWidget

# The server sends pan/tilt as offsets from centre. Higher pan angles are
# assumed to look LEFT; flip this if the drawings come out mirrored.
POSITIVE_BEARING_IS_LEFT = True
# Rough height of the ultrasonic sensor (on the head) above the floor.
SENSOR_HEIGHT_CM = 15
# 3D points lower than this are the floor, not objects.
FLOOR_CUTOFF_CM = 4
# Half-width of the ultrasonic beam. The sensor reports the nearest thing
# anywhere in that cone, so a level scan still "sees" the floor where the
# bottom edge of the cone meets it.
SENSOR_CONE_DEG = 8

BACKGROUND = QColor("#0b1a12")
GRID = QColor("#2f6b45")
GRID_TEXT = QColor("#6fbf8a")
TEXT = QColor("#e0f2e9")
OUTLINE = QColor("#3ddc84")
OBJECT_COLOR = QColor("#80d8ff")


def distance_color(cm, max_range):
    # Red = close, yellow = mid, green = far
    frac = cm / max_range
    if frac < 0.33:
        return QColor("#ff5252")
    if frac < 0.66:
        return QColor("#ffd740")
    return QColor("#69f0ae")


def to_xyz(bearing, elevation, cm):
    b = math.radians(bearing if POSITIVE_BEARING_IS_LEFT else -bearing)
    e = math.radians(elevation)
    flat = cm * math.cos(e)
    return (-flat * math.sin(b), flat * math.cos(b), SENSOR_HEIGHT_CM + cm * math.sin(e))


def describe_bearing(bearing):
    if abs(bearing) < 3:
        return "ahead"
    left = (bearing > 0) == POSITIVE_BEARING_IS_LEFT
    return "%d° %s" % (round(abs(bearing)), "left" if left else "right")


class RadarObject:
    def __init__(self, label, points):
        self.label = label
        self.points = points  # list of (x, y, z)
        xs, ys, zs = zip(*points)
        self.min = (min(xs), min(ys), min(zs))
        self.max = (max(xs), max(ys), max(zs))
        self.center = (sum(xs) / len(xs), sum(ys) / len(ys), sum(zs) / len(zs))
        self.distance = min(math.hypot(x, y) for x, y, _ in points)
        self.bearing = math.degrees(math.atan2(-self.center[0], self.center[1]))
        if not POSITIVE_BEARING_IS_LEFT:
            self.bearing = -self.bearing
        # Width across the robot's view, not along x: points spread sideways
        # around the centre direction.
        angles = [math.atan2(-x, y) for x, y, _ in points]
        self.width = 2 * self.distance * math.tan((max(angles) - min(angles)) / 2) if len(points) > 1 else 0
        self.height = self.max[2] - self.min[2]

    def kind(self):
        if self.width > 80:
            return "wall / large surface"
        return "object"

    def summary(self, three_d):
        text = "%s: %s, ~%d cm wide" % (self.label, self.kind(), round(self.width))
        if three_d:
            text += ", ~%d cm tall" % round(self.height)
        return text + ", %d cm away, %s" % (round(self.distance), describe_bearing(self.bearing))


def _is_cone_floor(group):
    # Flat cluster sitting right where the bottom of the cone meets the floor
    tilts = sorted(t for _, t, _, _ in group)
    ranges = sorted(cm for _, _, cm, _ in group)
    tilt, cm = tilts[len(tilts) // 2], ranges[len(ranges) // 2]
    down = SENSOR_CONE_DEG - tilt  # steepest downward angle inside the cone
    if down <= 0:
        return False
    floor_cm = SENSOR_HEIGHT_CM / math.sin(math.radians(min(89, down)))
    return abs(cm - floor_cm) < 0.12 * floor_cm


def find_objects(samples, min_points, pan_gap, tilt_gap, drop_floor=True):
    """Groups readings into objects. samples: (pan, tilt, cm, (x, y, z)).

    Two readings are the same surface if they're neighbours in the scan
    (close in pan and tilt) and at a similar distance - a jump in distance
    between neighbours is an edge. That suits this sensor better than plain
    3D distance: the scan rows are far apart, and its wide cone blurs
    things sideways rather than in depth."""
    n = len(samples)
    parent = list(range(n))

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i in range(n):
        pan_i, tilt_i, cm_i, _ = samples[i]
        for j in range(i + 1, n):
            pan_j, tilt_j, cm_j, _ = samples[j]
            if (abs(pan_i - pan_j) <= pan_gap and abs(tilt_i - tilt_j) <= tilt_gap
                    and abs(cm_i - cm_j) <= max(6.0, 0.12 * min(cm_i, cm_j))):
                parent[root(i)] = root(j)
    groups = {}
    for i in range(n):
        groups.setdefault(root(i), []).append(samples[i])
    clusters = []
    for group in groups.values():
        if len(group) < min_points:
            continue
        g = [xyz for _, _, _, xyz in group]
        zs = [z for _, _, z in g]
        if drop_floor and max(zs) - min(zs) < 5:
            if max(zs) < 10:
                continue  # low and flat: a patch of floor
            if _is_cone_floor(group):
                continue
        clusters.append(g)
    clusters.sort(key=lambda g: min(math.hypot(x, y) for x, y, _ in g))
    return [RadarObject(chr(ord("A") + i), g) for i, g in enumerate(clusters[:26])]


class RadarWidget(QWidget):
    """Top-down 2D fan: the robot sits at the bottom centre, straight ahead is up."""

    def __init__(self, parent=None):
        super(RadarWidget, self).__init__(parent)
        self.setMinimumHeight(260)
        self.readings = {}  # bearing -> list of cm (several sweeps hit each angle)
        self.max_range = 150
        self.title = ""
        self.objects = []

    def clear(self):
        self.readings = {}
        self.objects = []
        self.title = ""
        self.update()

    def add_point(self, bearing, distance):
        self.readings.setdefault(bearing, []).append(distance)
        self.update()

    def set_max_range(self, cm):
        self.max_range = cm
        self.update_objects()

    def points(self):
        # One distance per bearing: median of that bearing's readings.
        result = []
        for bearing in sorted(self.readings):
            values = sorted(self.readings[bearing])
            result.append((bearing, values[len(values) // 2]))
        return result

    def update_objects(self):
        bearings = sorted(self.readings)
        spacing = min((b - a for a, b in zip(bearings, bearings[1:])), default=5)
        samples = [(b, 0, d, to_xyz(b, 0, d)) for b, d in self.points() if d < self.max_range * 0.98]
        self.objects = find_objects(samples, min_points=2, pan_gap=spacing + 0.5, tilt_gap=0,
                                    drop_floor=False)  # a level sweep can't tell floor from wall
        self.update()
        return self.objects

    def paintEvent(self, event):
        painter = QPainter(self)
        self.paint(painter, self.width(), self.height())
        painter.end()

    def to_image(self, width=900, height=560):
        image = QImage(width, height, QImage.Format_RGB32)
        painter = QPainter(image)
        self.paint(painter, width, height)
        painter.end()
        return image

    def paint(self, painter, w, h):
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(0, 0, w, h, BACKGROUND)
        margin = 30
        radius = max(10, min(w / 2 - margin, h - margin * 2))
        origin = QPointF(w / 2, h - margin)
        scale = radius / self.max_range

        def at(bearing, cm):
            x, y, _ = to_xyz(bearing, 0, min(cm, self.max_range))
            return QPointF(origin.x() + x * scale, origin.y() - y * scale)

        # Range rings and bearing spokes
        painter.setFont(QFont("Arial", 9))
        for i in range(1, 5):
            r = radius * i / 4
            painter.setPen(QPen(GRID, 1))
            painter.drawArc(QRectF(origin.x() - r, origin.y() - r, 2 * r, 2 * r), 0, 180 * 16)
            painter.setPen(GRID_TEXT)
            painter.drawText(QPointF(origin.x() + 4, origin.y() - r + 12), "%d cm" % round(self.max_range * i / 4))
        painter.setPen(QPen(GRID, 1, Qt.DashLine))
        for bearing in range(-90, 91, 30):
            painter.drawLine(origin, at(bearing, self.max_range))

        pts = self.points()
        if pts:
            # Scanned area, filled out to each reading
            outline = QPolygonF([origin] + [at(b, d) for b, d in pts])
            painter.setPen(QPen(OUTLINE, 2))
            painter.setBrush(QColor(61, 220, 132, 60))
            painter.drawPolygon(outline)
            # Dots, hollow = nothing in range
            for b, d in pts:
                color = distance_color(d, self.max_range)
                painter.setPen(QPen(color, 2))
                painter.setBrush(Qt.NoBrush if d >= self.max_range else color)
                painter.drawEllipse(at(b, d), 4, 4)

        # Object labels
        painter.setFont(QFont("Arial", 10, QFont.Bold))
        for obj in self.objects:
            p = QPointF(origin.x() + obj.center[0] * scale, origin.y() - obj.center[1] * scale)
            painter.setPen(QPen(OBJECT_COLOR, 1, Qt.DashLine))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(p, 14, 14)
            painter.setPen(OBJECT_COLOR)
            painter.drawText(QPointF(p.x() + 16, p.y() - 10), obj.label)

        # Robot marker
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("#ffffff"))
        painter.drawEllipse(origin, 6, 6)

        painter.setPen(TEXT)
        painter.setFont(QFont("Arial", 10, QFont.Bold))
        painter.drawText(QPointF(10, 18), self.title or "Radar scan")


class Radar3DWidget(QWidget):
    """Rotatable 3D ping cloud. Drag to orbit, scroll to zoom. New pings
    flash bright and fade in like a sonar display."""

    FLASH_S = 1.2

    def __init__(self, parent=None):
        super(Radar3DWidget, self).__init__(parent)
        self.setMinimumHeight(260)
        self.pings = []  # (x, y, z, cm, time added)
        self.angles = []  # (pan, tilt) of each ping
        self.row_step = 8  # degrees between tilt rows, set by the client per scan
        self.max_range = 150
        self.title = ""
        self.objects = []
        self.yaw = -35.0    # orbit angles, degrees
        self.pitch = 25.0
        self.zoom = 1.0
        self.drag_from = None
        # Repaints while there are still pings fading in
        self.anim = QTimer(self)
        self.anim.timeout.connect(self._tick)
        self.anim.start(33)

    def _tick(self):
        if self.pings and time.monotonic() - self.pings[-1][4] < self.FLASH_S:
            self.update()

    def clear(self):
        self.pings = []
        self.angles = []
        self.objects = []
        self.title = ""
        self.update()

    def add_point(self, pan, tilt, cm):
        x, y, z = to_xyz(pan, tilt, cm)
        self.pings.append((x, y, z, cm, time.monotonic()))
        self.angles.append((pan, tilt))
        self.update()

    def set_max_range(self, cm):
        self.max_range = cm
        self.update_objects()

    def update_objects(self):
        # In range and above the floor - candidates for objects
        samples = [(pan, tilt, cm, (x, y, z))
                   for (x, y, z, cm, _), (pan, tilt) in zip(self.pings, self.angles)
                   if cm < self.max_range and z > FLOOR_CUTOFF_CM]
        self.objects = find_objects(samples, min_points=4, pan_gap=4, tilt_gap=self.row_step + 1)
        self.update()
        return self.objects

    # --- mouse: drag to orbit, wheel to zoom -------------------------------
    def mousePressEvent(self, event):
        self.drag_from = event.pos()

    def mouseMoveEvent(self, event):
        if self.drag_from is None:
            return
        delta = event.pos() - self.drag_from
        self.drag_from = event.pos()
        self.yaw += delta.x() * 0.5
        self.pitch = max(-5.0, min(89.0, self.pitch + delta.y() * 0.5))
        self.update()

    def mouseReleaseEvent(self, event):
        self.drag_from = None

    def wheelEvent(self, event):
        self.zoom = max(0.3, min(4.0, self.zoom * (1.1 if event.angleDelta().y() > 0 else 1 / 1.1)))
        self.update()

    # --- projection --------------------------------------------------------
    def _projector(self, w, h):
        # Orbit around a point halfway out into the scanned area.
        cx, cy, cz = 0.0, self.max_range * 0.5, self.max_range * 0.15
        yaw, pitch = math.radians(self.yaw), math.radians(self.pitch)
        cam_dist = self.max_range * 3.0
        focal = min(w, h) * 1.7 * self.zoom

        def project(x, y, z):
            x, y, z = x - cx, y - cy, z - cz
            # rotate around the vertical axis, then tilt the view down
            x, y = x * math.cos(yaw) - y * math.sin(yaw), x * math.sin(yaw) + y * math.cos(yaw)
            y, z = y * math.cos(pitch) - z * math.sin(pitch), y * math.sin(pitch) + z * math.cos(pitch)
            depth = cam_dist + y
            if depth < 1:
                return None
            s = focal / depth
            return QPointF(w / 2 + x * s, h / 2 - z * s), s, depth
        return project

    def paintEvent(self, event):
        painter = QPainter(self)
        self.paint(painter, self.width(), self.height())
        painter.end()

    def to_image(self, width=900, height=560):
        image = QImage(width, height, QImage.Format_RGB32)
        painter = QPainter(image)
        self.paint(painter, width, height, final=True)
        painter.end()
        return image

    def paint(self, painter, w, h, final=False):
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(0, 0, w, h, BACKGROUND)
        project = self._projector(w, h)
        r = self.max_range

        def line(a, b, pen):
            pa, pb = project(*a), project(*b)
            if pa and pb:
                painter.setPen(pen)
                painter.drawLine(pa[0], pb[0])

        # Floor grid, every quarter of the range
        step = r / 4
        grid_pen = QPen(GRID, 1)
        for i in range(-4, 5):
            line((i * step, 0, 0), (i * step, r, 0), grid_pen)
        for i in range(0, 5):
            line((-r, i * step, 0), (r, i * step, 0), grid_pen)
        painter.setFont(QFont("Arial", 8))
        for i in range(1, 5):
            p = project(r + 8, i * step, 0)
            if p:
                painter.setPen(GRID_TEXT)
                painter.drawText(p[0], "%d cm" % round(i * step))

        # Scanned field of view (edges of the pan range at sensor height)
        fov_pen = QPen(GRID, 1, Qt.DashLine)
        for bearing in (-35, 35):
            line(to_xyz(bearing, 0, 0), to_xyz(bearing, 0, r), fov_pen)

        # Robot: a small hexagon on the floor plus a mast up to the sensor
        hexagon = [project(12 * math.cos(math.radians(a)), 12 * math.sin(math.radians(a)), 0)
                   for a in range(0, 360, 60)]
        if all(hexagon):
            painter.setPen(QPen(TEXT, 1))
            painter.setBrush(QColor(255, 255, 255, 60))
            painter.drawPolygon(QPolygonF([p[0] for p in hexagon]))
        line((0, 0, 0), (0, 0, SENSOR_HEIGHT_CM), QPen(TEXT, 2))

        # Pings, far ones first so near ones draw on top
        now = time.monotonic()
        drawn = []
        for x, y, z, cm, added in self.pings:
            if cm >= r:
                continue
            p = project(x, y, z)
            if p:
                drawn.append((p[2], p, cm, added))
        drawn.sort(key=lambda d: -d[0])
        painter.setPen(Qt.NoPen)
        for _, (pt, s, _), cm, added in drawn:
            age = 1.0 if final else min(1.0, (now - added) / self.FLASH_S)
            color = QColor(distance_color(cm, r))
            size = max(1.5, 1.5 * s)
            if age < 1.0:
                # Fresh ping: white-hot halo that cools to its distance colour
                halo = QColor(255, 255, 255, int(160 * (1 - age)))
                painter.setBrush(halo)
                painter.drawEllipse(pt, size * (3 - 2 * age), size * (3 - 2 * age))
            color.setAlpha(200)
            painter.setBrush(color)
            painter.drawEllipse(pt, size, size)

        # Beam line to the newest ping while scanning
        if self.pings and not final:
            x, y, z, cm, added = self.pings[-1]
            fade = 1 - min(1.0, (now - added) / 0.5)
            if fade > 0 and cm < r:
                line((0, 0, SENSOR_HEIGHT_CM), (x, y, z), QPen(QColor(61, 220, 132, int(180 * fade)), 1))

        # Object boxes
        painter.setBrush(Qt.NoBrush)
        box_pen = QPen(OBJECT_COLOR, 1, Qt.DashLine)
        for obj in self.objects:
            (x0, y0, z0), (x1, y1, z1) = obj.min, obj.max
            corners = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
            for i, a in enumerate(corners):
                for b in corners[i + 1:]:
                    if sum(u != v for u, v in zip(a, b)) == 1:  # box edges only
                        line(a, b, box_pen)
            top = project(obj.center[0], obj.center[1], z1 + 6)
            if top:
                painter.setPen(OBJECT_COLOR)
                painter.setFont(QFont("Arial", 10, QFont.Bold))
                painter.drawText(top[0], obj.label)

        painter.setPen(TEXT)
        painter.setFont(QFont("Arial", 10, QFont.Bold))
        painter.drawText(QPointF(10, 18), self.title or "3D ping scan")
        painter.setFont(QFont("Arial", 8))
        painter.setPen(GRID_TEXT)
        painter.drawText(QPointF(10, h - 10), "%d pings  -  drag to rotate, scroll to zoom" % len(self.pings))
