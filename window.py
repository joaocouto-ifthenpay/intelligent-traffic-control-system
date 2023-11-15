import numpy as np
import pygame
from pygame.draw import polygon


# # For debugging purposes
# DRAW_VEHICLE_IDS = True
# DRAW_ROAD_IDS = False
# FILL_POLYGONS = True


class Window:
    def __init__(self, simulation):
        self._width = 700
        self._height = 600

        self.closed: bool = False
        self._sim = simulation

        self._background_color = (235, 235, 235)
        self._screen = pygame.display.set_mode((self._width, self._height))
        pygame.display.set_caption('Sistema Multi-Agente de Controlo de Tráfego')
        pygame.display.flip()
        pygame.font.init()
        font = f'freesans'
        self._text_font = pygame.font.SysFont(font, 18)
        self._zoom = 5
        self._offset = (0, 0)
        self._mouse_last = (0, 0)
        self._mouse_down = False

    def update(self) -> None:
        self._draw()
        pygame.display.update()
        for event in pygame.event.get():
            # Quit program if window is closed
            if event.type == pygame.QUIT:
                self.closed = True

    def _convert(self, x, y=None):
        """Converts simulation coordinates to screen coordinates"""
        if isinstance(x, list):
            return [self._convert(e[0], e[1]) for e in x]
        if isinstance(x, tuple):
            return self._convert(*x)
        return (int(self._width / 2 + (x + self._offset[0]) * self._zoom),
                int(self._height / 2 + (y + self._offset[1]) * self._zoom))

    def _inverse_convert(self, x, y=None):
        """Converts screen coordinates to simulation coordinates"""
        if isinstance(x, list):
            return [self._convert(e[0], e[1]) for e in x]
        if isinstance(x, tuple):
            return self._convert(*x)
        return (int(-self._offset[0] + (x - self._width / 2) / self._zoom),
                int(-self._offset[1] + (y - self._height / 2) / self._zoom))

    def _rotated_box(self, pos, size, angle=None, cos=None, sin=None, centered=True,
                     color=(82, 166, 232)):
        """Draws a rectangle center at *pos* with size *size* rotated anti-clockwise by *angle*."""

        def vertex(e1, e2):
            return (x + (e1 * l * cos + e2 * h * sin) / 2,
                    y + (e1 * l * sin - e2 * h * cos) / 2)

        x, y = pos
        l, h = size
        if angle:
            cos, sin = np.cos(angle), np.sin(angle)
        if centered:
            points = self._convert([vertex(*e) for e in [(-1, -1), (-1, 1), (1, 1), (1, -1)]])
        else:
            points = self._convert([vertex(*e) for e in [(0, -1), (0, 1), (2, 1), (2, -1)]])

        polygon(self._screen, color, points)

        # # For debugging purposes
        # width = 0 if FILL_POLYGONS else 2
        # x1, x2 = points[0][0], points[2][0]
        # y1, y2 = points[0][1], points[2][1]
        # screen_x = x1 + (x2 - x1) / 2
        # screen_y = y1 + (y2 - y1) / 2
        # polygon(self._screen, color, points, width)
        # return screen_x, screen_y

    def _draw_arrow(self, pos, size, angle=None, cos=None, sin=None, color=(85, 85, 85)) -> None: #180,180,180
        if angle:
            cos, sin = np.cos(angle), np.sin(angle)
        self._rotated_box(pos,
                          size,
                          cos=(cos - sin) / np.sqrt(2),
                          sin=(cos + sin) / np.sqrt(2),
                          color=color,
                          centered=False)
        self._rotated_box(pos,
                          size,
                          cos=(cos + sin) / np.sqrt(2),
                          sin=(sin - cos) / np.sqrt(2),
                          color=color,
                          centered=False)

    def _draw_roads(self) -> None:
        # road_index_coordinates = [] # For debugging purposes
        for road in self._sim.traffic_controllers:
            # Draw road background
            self._rotated_box(
                road.start,
                (road.length, 3.7),
                cos=road.angle_cos,
                sin=road.angle_sin,
                color=(85, 85, 85),
                centered=False
            )

            # # For debugging purposes
            # road_index_coordinates.append((road.index, screen_x, screen_y))

            # Draw road arrow
            if road.length > 5:
                for i in np.arange(-0.5 * road.length, 0.5 * road.length, 10):
                    pos = (road.start[0] + (road.length / 2 + i + 3) * road.angle_cos,
                           road.start[1] + (road.length / 2 + i + 3) * road.angle_sin)
                    self._draw_arrow(pos, (-1.25, 0.2), cos=road.angle_cos, sin=road.angle_sin)

        # # For debugging purposes
        # if DRAW_ROAD_IDS:
        #     # For debugging purposes
        #     for cords in road_index_coordinates:
        #         text_road_index = self._text_font.render(f'{cords[0]}', True, (0, 0, 0))
        #         self._screen.blit(text_road_index, (cords[1] - 5, cords[2] - 5))

    def _draw_vehicle(self, vehicle, road) -> None:
        l, h = vehicle.length, vehicle.width
        sin, cos = road.angle_sin, road.angle_cos
        x = road.start[0] + cos * vehicle.x
        y = road.start[1] + sin * vehicle.x
        self._rotated_box((x, y), (l, h), cos=cos, sin=sin, centered=True)

        #radius = vehicle.width*4  # Usando a metade da largura como raio para representar um círculo
        #x = road.start[0] + road.angle_cos * vehicle.x
        #y = road.start[1] + road.angle_sin * vehicle.x

        #center = self._convert(x, y)
        #pygame.draw.circle(self._screen, (255, 0, 0), center, int(radius))

        # # For debugging purposes
        # screen_x, screen_y = self._rotated_box((x, y), (l, h), cos=cos, sin=sin, centered=True)
        # if DRAW_VEHICLE_IDS:
        #     text_road_index = self._text_font.render(f'{vehicle.index}', True, (255, 255, 255),
        #                                              (0, 0, 0))
        #     self._screen.blit(text_road_index, (screen_x - 5, screen_y - 5))

    def _draw_vehicles(self) -> None:
        for i in self._sim.non_empty_roads:
            road = self._sim.traffic_controllers[i]
            for vehicle in road.vehicles:
                self._draw_vehicle(vehicle, road)

    def _draw_signals(self) -> None:
        for signal in self._sim.traffic_signals:
            for i in range(len(signal.traffic_controllers)):
                red, green = (255, 0, 0), (0, 255, 0)
                if signal.current_cycle == (False, False):
                    # Temp state, yellow color
                    yellow = (255, 255, 0)
                    color = yellow if signal.cycle[signal.current_cycle_index - 1][i] else red
                else:
                    color = green if signal.current_cycle[i] else red
                for road in signal.traffic_controllers[i]:
                    a = 0
                    position = ((1 - a) * road.end[0] + a * road.start[0],
                                (1 - a) * road.end[1] + a * road.start[1])
                    self._rotated_box(position, (1, 3),
                                      cos=road.angle_cos, sin=road.angle_sin, color=color)

    def _draw_status(self):
        def render(text, color=(0, 0, 0), background=self._background_color):
            return self._text_font.render(text, True, color, background)

        t = render(f'Tempo: {self._sim.t:.1f}')
        if self._sim.max_gen:
            n_max_gen = render(f'Veículos gerados máx: {self._sim.max_gen}')
            self._screen.blit(n_max_gen, (10, 50))
        n_vehicles_generated = render(f'Veículos gerados: {self._sim.n_vehicles_generated}')
        n_vehicles_on_map = render(f'Veículos na estrada: {self._sim.n_vehicles_on_map}')
        average_wait_time = render(f'Tempo de espera: {self._sim.current_average_wait_time:.1f}')
        self._screen.blit(t, (10, 20))
        self._screen.blit(n_vehicles_generated, (10, 70))
        self._screen.blit(n_vehicles_on_map, (10, 90))
        self._screen.blit(average_wait_time, (10, 120))

    def _draw(self):
        self._screen.fill(self._background_color)
        self._draw_roads()
        self._draw_vehicles()
        self._draw_signals()
        self._draw_status()
