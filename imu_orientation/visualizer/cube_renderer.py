"""
OpenGL 3D cube renderer for orientation visualization.
"""

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import math


# Cube vertices (centered at origin)
VERTICES = [
    # Front face
    (-1, -1,  1),
    ( 1, -1,  1),
    ( 1,  1,  1),
    (-1,  1,  1),
    # Back face
    (-1, -1, -1),
    ( 1, -1, -1),
    ( 1,  1, -1),
    (-1,  1, -1),
]

# Edges connecting vertices
EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 0),  # Front
    (4, 5), (5, 6), (6, 7), (7, 4),  # Back
    (0, 4), (1, 5), (2, 6), (3, 7),  # Sides
]

# Faces with colors (vertex indices and color)
FACES = [
    ([0, 1, 2, 3], (1.0, 0.0, 0.0)),  # Front - Red
    ([4, 7, 6, 5], (0.0, 1.0, 0.0)),  # Back - Green
    ([3, 2, 6, 7], (0.0, 0.0, 1.0)),  # Top - Blue
    ([0, 4, 5, 1], (1.0, 1.0, 0.0)),  # Bottom - Yellow
    ([0, 3, 7, 4], (1.0, 0.0, 1.0)),  # Left - Magenta
    ([1, 5, 6, 2], (0.0, 1.0, 1.0)),  # Right - Cyan
]


class CubeRenderer:
    """Renders a 3D cube with orientation from IMU data."""

    def __init__(self, width: int = 800, height: int = 600):
        self.width = width
        self.height = height
        self.roll = 0.0
        self.pitch = 0.0
        self.heading = 0.0
        self.running = True

    def init(self) -> bool:
        """Initialize pygame and OpenGL."""
        pygame.init()
        pygame.display.set_mode((self.width, self.height), DOUBLEBUF | OPENGL)
        pygame.display.set_caption("IMU Orientation Visualizer")

        # OpenGL setup
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

        # Light position
        glLightfv(GL_LIGHT0, GL_POSITION, (5.0, 5.0, 10.0, 1.0))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.3, 0.3, 0.3, 1.0))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.8, 0.8, 0.8, 1.0))

        # Perspective
        glMatrixMode(GL_PROJECTION)
        gluPerspective(45, self.width / self.height, 0.1, 50.0)
        glMatrixMode(GL_MODELVIEW)

        return True

    def set_orientation(self, roll: float, pitch: float, heading: float):
        """Set the current orientation in degrees."""
        self.roll = roll
        self.pitch = pitch
        self.heading = heading

    def _draw_cube(self):
        """Draw the cube with colored faces."""
        # Draw filled faces
        glBegin(GL_QUADS)
        for vertices, color in FACES:
            glColor3fv(color)
            for vertex in vertices:
                glVertex3fv(VERTICES[vertex])
        glEnd()

        # Draw edges in black
        glDisable(GL_LIGHTING)
        glColor3f(0.0, 0.0, 0.0)
        glLineWidth(2.0)
        glBegin(GL_LINES)
        for edge in EDGES:
            for vertex in edge:
                glVertex3fv(VERTICES[vertex])
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_axes(self):
        """Draw coordinate axes for reference."""
        glDisable(GL_LIGHTING)
        glLineWidth(3.0)

        axis_length = 2.5

        glBegin(GL_LINES)
        # X axis - Red
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(0, 0, 0)
        glVertex3f(axis_length, 0, 0)

        # Y axis - Green
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(0, 0, 0)
        glVertex3f(0, axis_length, 0)

        # Z axis - Blue
        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, axis_length)
        glEnd()

        glEnable(GL_LIGHTING)

    def _draw_text(self):
        """Draw orientation values as text overlay."""
        # Switch to 2D for text
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(0, self.width, 0, self.height, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()

        glDisable(GL_LIGHTING)
        glDisable(GL_DEPTH_TEST)

        # Render text using pygame (simple approach)
        # Note: For production, use a proper text rendering method

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)

        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()

    def render(self):
        """Render one frame."""
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glClearColor(0.2, 0.2, 0.3, 1.0)

        glLoadIdentity()

        # Camera position
        glTranslatef(0.0, 0.0, -7.0)

        # Draw fixed reference axes
        glPushMatrix()
        glTranslatef(-3.0, -2.0, 0.0)
        glScalef(0.5, 0.5, 0.5)
        self._draw_axes()
        glPopMatrix()

        # Apply orientation rotations
        # Order: Heading (yaw around Z), Pitch (around X), Roll (around Y)
        # Note: OpenGL uses different axis conventions, adjust as needed
        glRotatef(-self.heading, 0, 1, 0)  # Yaw around Y (up)
        glRotatef(self.pitch, 1, 0, 0)      # Pitch around X
        glRotatef(-self.roll, 0, 0, 1)      # Roll around Z

        # Draw the oriented cube
        self._draw_cube()

        pygame.display.flip()

    def process_events(self) -> bool:
        """Process pygame events. Returns False if window closed."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                return False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                    return False
                elif event.key == pygame.K_r:
                    # Reset orientation
                    self.roll = 0.0
                    self.pitch = 0.0
                    self.heading = 0.0
        return True

    def cleanup(self):
        """Clean up pygame."""
        pygame.quit()


def main():
    """Test the cube renderer with keyboard input."""
    renderer = CubeRenderer()
    if not renderer.init():
        return

    clock = pygame.time.Clock()

    roll, pitch, heading = 0.0, 0.0, 0.0

    print("Use arrow keys to rotate, R to reset, ESC to quit")

    while renderer.running:
        if not renderer.process_events():
            break

        # Keyboard control for testing
        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT]:
            heading -= 2.0
        if keys[pygame.K_RIGHT]:
            heading += 2.0
        if keys[pygame.K_UP]:
            pitch += 2.0
        if keys[pygame.K_DOWN]:
            pitch -= 2.0
        if keys[pygame.K_a]:
            roll -= 2.0
        if keys[pygame.K_d]:
            roll += 2.0

        # Keep heading in 0-360 range
        heading = heading % 360

        renderer.set_orientation(roll, pitch, heading)
        renderer.render()

        # Update window title with values
        pygame.display.set_caption(
            f"Roll: {roll:.1f}  Pitch: {pitch:.1f}  Heading: {heading:.1f}"
        )

        clock.tick(60)

    renderer.cleanup()


if __name__ == '__main__':
    main()
