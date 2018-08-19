import copy
from utils import Utils

class Plan:
    C1 = 0
    C2 = 1
    C3 = 2
    NB_MAX_COMPONENTS = 3

    def __init__(self, components, obstacle = None, translation_vector = None, safe_swept_area = None, push_pose = None):

        self.components = components
        self.obstacle = obstacle
        self.translation_vector = translation_vector
        self.safe_swept_area = safe_swept_area
        self.push_pose = push_pose

        # If this is a plan with only one path component (avoid all obstacles)
        self.cost = components[Plan.C1].cost
        self.min_cost = None

        # If this is a plan with more three components (manipulate an obstacle)
        if len(self.components) == Plan.NB_MAX_COMPONENTS:
            self.cost = components[Plan.C1].cost + components[Plan.C2].cost + components[Plan.C3].cost
            self.min_cost = components[Plan.C2].cost + components[Plan.C3].cost


        self.next_step = None
        if self.components[Plan.C1].path.poses:
            self.next_step = self.components[Plan.C1].path.poses[0]
        self._step_counter = 0
        self.next_step_component = Plan.C1

    def is_plan_colliding(self, multilayered_map):
        is_c1_colliding = self.components[0].is_path_colliding(multilayered_map, True)

        if is_c1_colliding:
            return True

        if len(self.components) == Plan.NB_MAX_COMPONENTS:
            for point in self.safe_swept_area:
                if (point not in self.obstacle.discretized_polygon and
                        multilayered_map.merged_occ_grid[point[0]][point[1]] == Utils.ROS_COST_LETHAL):
                    return True

            map_mod = copy.deepcopy(multilayered_map)
            map_mod.manually_move_obstacle(self.obstacle.obstacle_id, self.translation_vector)
            is_c3_colliding = self.components[2].is_path_colliding(map_mod)

            if is_c3_colliding:
                return True

        return False

    def get_next_step(self):
        # If there still are poses to go to in the current plan component
        if self._step_counter + 1 < len(self.components[self.next_step_component].path.poses):
            self._step_counter = self._step_counter + 1
            self.next_step = self.components[self.next_step_component].path.poses[self._step_counter]
        else:
            # Else if there are still other components after the current plan component
            if self.next_step_component + 1 < len(self.components):
                self.next_step_component = self.next_step_component + 1
                self._step_counter = 0
                self.next_step = self.components[self.next_step_component].path.poses[self._step_counter]
            else:
                self.next_step = None
        return self.next_step
