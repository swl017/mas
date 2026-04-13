import numpy as np
from typing import List, Tuple, Dict, Optional, Set
import scipy.optimize as optimize

# Note: This would normally be imported with:
# import pyceres as ceres
# But for this implementation, I'll create a wrapper class to simulate Ceres functionality

class CeresOptimizer:
    """
    Wrapper class for Ceres Solver functionality.
    In a real implementation, you would use pyceres instead.
    """
    def __init__(self):
        pass

    def create_problem(self):
        return CeresProblem()

    def solve(self, problem, options=None):
        if options is None:
            options = {
                'max_num_iterations': 100,
                'linear_solver_type': 'DENSE_QR',
                'minimizer_progress_to_stdout': True
            }

        # In reality, this would call Ceres, but here we'll use scipy
        return problem.solve(options)

class CeresProblem:
    """
    Simulates a Ceres Problem object. In real implementation,
    this would be replaced by the actual Ceres Problem class.
    """
    def __init__(self):
        self.cost_functions = []
        self.parameters = []
        self.parameter_blocks = []
        self.hessian_approximation = None

    def add_residual_block(self, cost_function, loss_function, *parameters):
        self.cost_functions.append(cost_function)
        for param in parameters:
            if param not in self.parameter_blocks:
                self.parameter_blocks.append(param)
                self.parameters.append(param)

    def solve(self, options):
        # Simplified optimization using scipy
        # In a real implementation, this would use Ceres Solver

        # Extract parameters to optimize
        x0 = np.concatenate([param.flatten() for param in self.parameters])

        # Define objective function for optimization
        def objective(x):
            # Update parameters
            idx = 0
            updated_params = []
            for param in self.parameters:
                param_size = param.size
                updated_param = x[idx:idx+param_size].reshape(param.shape)
                updated_params.append(updated_param)
                idx += param_size

            # Compute total residual
            residual = 0
            for cost_fn in self.cost_functions:
                res = cost_fn.evaluate(updated_params)
                residual += np.sum(res**2)

            return residual

        # Define Jacobian function
        def jacobian(x):
            # Update parameters
            idx = 0
            updated_params = []
            for param in self.parameters:
                param_size = param.size
                updated_param = x[idx:idx+param_size].reshape(param.shape)
                updated_params.append(updated_param)
                idx += param_size

            # Compute Jacobian using finite differences
            epsilon = 1e-8
            x_length = len(x)
            jac = np.zeros(x_length)

            for i in range(x_length):
                x_plus = x.copy()
                x_plus[i] += epsilon
                f_plus = objective(x_plus)

                x_minus = x.copy()
                x_minus[i] -= epsilon
                f_minus = objective(x_minus)

                jac[i] = (f_plus - f_minus) / (2 * epsilon)

            return jac

        # Perform optimization with BFGS method to get Hessian approximation
        result = optimize.minimize(
            objective,
            x0,
            method='BFGS',
            jac=jacobian,
            options={'maxiter': options.get('max_num_iterations', 100)}
        )

        # Store the Hessian approximation for uncertainty estimation
        # In BFGS, the inverse Hessian is approximated during optimization
        self.hessian_approximation = result.hess_inv

        # Update parameters with optimized values
        idx = 0
        for i, param in enumerate(self.parameters):
            param_size = param.size
            self.parameters[i] = result.x[idx:idx+param_size].reshape(param.shape)
            idx += param_size

        return {'success': result.success, 'iterations': result.nit, 'final_cost': result.fun}

    def get_covariance_matrix(self):
        """
        Get the covariance matrix from the optimization result.
        In a real implementation, this would use Ceres's covariance estimation.

        Returns:
            Covariance matrix for the optimized parameters
        """
        if self.hessian_approximation is None:
            raise ValueError("Problem must be solved before computing covariance")

        # The inverse Hessian approximates the covariance matrix
        return self.hessian_approximation

class ReprojectionCostFunction:
    """
    Cost function for reprojection error.
    """
    def __init__(self, camera, point_2d):
        self.camera = camera
        self.point_2d = point_2d

    def evaluate(self, parameters):
        # Parameters should contain a single 3D point
        point_3d = parameters[0]

        # Project 3D point to image plane
        projected_point = self.camera.project_point(point_3d)

        # Compute reprojection error
        error = projected_point - self.point_2d

        return error

class Camera:
    def __init__(self,
                 camera_id: int,
                 image_width: int,
                 image_height: int,
                 intrinsic_matrix: np.ndarray,
                 extrinsic_matrix: np.ndarray):
        """
        Initialize a camera with its parameters.

        Args:
            camera_id: Unique identifier for the camera
            image_width: Width of the image in pixels
            image_height: Height of the image in pixels
            intrinsic_matrix: 3x3 camera intrinsic matrix K
            extrinsic_matrix: 4x4 camera extrinsic matrix [R|t]
        """
        self.camera_id = camera_id
        self.image_width = image_width
        self.image_height = image_height
        self.intrinsic_matrix = intrinsic_matrix
        self.extrinsic_matrix = extrinsic_matrix

        # Calculate projection matrix P = K[R|t]
        self.projection_matrix = intrinsic_matrix @ extrinsic_matrix[:3, :]

    def project_point(self, point_3d: np.ndarray) -> np.ndarray:
        """
        Project a 3D point to the image plane.

        Args:
            point_3d: 3D point in world coordinates (x, y, z)

        Returns:
            2D point in image coordinates (u, v)
        """
        # Convert to homogeneous coordinates
        point_3d_h = np.append(point_3d, 1)

        # Project 3D point to image plane
        point_2d_h = self.projection_matrix @ point_3d_h

        # Convert back from homogeneous coordinates
        point_2d = point_2d_h[:2] / point_2d_h[2]

        return point_2d

    def get_ray(self, point_2d: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute a ray from the camera center through a given 2D point on the image plane.

        Args:
            point_2d: 2D point in image coordinates (u, v)

        Returns:
            ray_origin: Camera center in world coordinates
            ray_direction: Direction of the ray in world coordinates
        """
        # Get camera center (camera position in world coordinates)
        # The camera center C is where RC + t = 0, so C = -R^T * t
        R = self.extrinsic_matrix[:3, :3]
        t = self.extrinsic_matrix[:3, 3]
        ray_origin = -np.linalg.inv(R) @ t

        # Compute ray direction in camera coordinates
        point_2d_h = np.append(point_2d, 1)
        ray_dir_camera = np.linalg.inv(self.intrinsic_matrix) @ point_2d_h

        # Transform ray direction to world coordinates
        ray_direction = np.linalg.inv(R) @ ray_dir_camera
        ray_direction = ray_direction / np.linalg.norm(ray_direction)

        return ray_origin, ray_direction

class BoundingBox:
    def __init__(self,
                 center: np.ndarray,
                 width: float,
                 height: float,
                 camera_id: int,
                 detection_id: int = -1):
        """
        Initialize a bounding box with its parameters.

        Args:
            center: Center of the bounding box in image coordinates (u, v)
            width: Width of the bounding box in pixels
            height: Height of the bounding box in pixels
            camera_id: ID of the camera that observed this bounding box
            detection_id: ID of the detection within a camera (for multiple detections)
        """
        self.center = center
        self.width = width
        self.height = height
        self.camera_id = camera_id
        self.detection_id = detection_id

        # Calculate the corners of the bounding box
        half_width = width / 2
        half_height = height / 2

        self.top_left = np.array([center[0] - half_width, center[1] - half_height])
        self.top_right = np.array([center[0] + half_width, center[1] - half_height])
        self.bottom_left = np.array([center[0] - half_width, center[1] + half_height])
        self.bottom_right = np.array([center[0] + half_width, center[1] + half_height])

        # These will be used for debugging and visualization
        self.corners = np.array([self.top_left, self.top_right,
                                self.bottom_right, self.bottom_left])

        # Initialize association information
        self.associated_target_id = None


class UncertainPoint3D:
    """
    Class representing a 3D point with uncertainty.

    The uncertainty is represented as a 3D Gaussian distribution
    with a covariance matrix.
    """
    def __init__(self,
               position: np.ndarray,
               covariance: np.ndarray = None):
        """
        Initialize a 3D point with uncertainty.

        Args:
            position: Mean position of the point (3D vector)
            covariance: 3x3 covariance matrix representing uncertainty
        """
        self.position = position

        # Initialize with identity covariance if not provided
        if covariance is None:
            self.covariance = np.eye(3)
        else:
            self.covariance = covariance

    def confidence_ellipsoid(self, confidence: float = 0.95) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get the confidence ellipsoid for this uncertain point.

        Args:
            confidence: Confidence level (e.g., 0.95 for 95% confidence)

        Returns:
            radii: Principal radii of the ellipsoid
            axes: Principal axes of the ellipsoid (as columns of a matrix)
        """
        # For a 3D Gaussian, chi-square with 3 degrees of freedom
        # determines the scaling factor for the ellipsoid
        chi2_val = {
            0.50: 2.37,  # 50% confidence
            0.68: 3.53,  # 68% confidence (~1 sigma)
            0.90: 6.25,  # 90% confidence
            0.95: 7.81,  # 95% confidence (~2 sigma)
            0.99: 11.34  # 99% confidence (~3 sigma)
        }

        # Get the nearest chi-square value
        nearest_conf = min(chi2_val.keys(), key=lambda x: abs(x - confidence))
        scale_factor = chi2_val[nearest_conf]

        # Get eigenvalues and eigenvectors of the covariance matrix
        eigenvalues, eigenvectors = np.linalg.eigh(self.covariance)

        # Compute the radii of the ellipsoid
        radii = np.sqrt(scale_factor * eigenvalues)

        return radii, eigenvectors

    def mahalanobis_distance(self, point: np.ndarray) -> float:
        """
        Compute the Mahalanobis distance from this uncertain point to another point.

        Args:
            point: 3D point to compute distance to

        Returns:
            Mahalanobis distance
        """
        diff = point - self.position
        inv_cov = np.linalg.inv(self.covariance)
        return np.sqrt(diff.T @ inv_cov @ diff)

    def probability_density(self, point: np.ndarray) -> float:
        """
        Compute the probability density at a given point.

        Args:
            point: 3D point to compute probability density at

        Returns:
            Probability density
        """
        diff = point - self.position
        inv_cov = np.linalg.inv(self.covariance)
        det_cov = np.linalg.det(self.covariance)

        # Multivariate Gaussian PDF
        norm_const = 1.0 / (np.sqrt((2 * np.pi) ** 3 * det_cov))
        return norm_const * np.exp(-0.5 * diff.T @ inv_cov @ diff)

    def __str__(self) -> str:
        """String representation of the uncertain point."""
        # Calculate standard deviations from the covariance matrix
        std_devs = np.sqrt(np.diag(self.covariance))

        return (f"Position: [{self.position[0]:.3f}, {self.position[1]:.3f}, {self.position[2]:.3f}]\n"
                f"Std Devs: [{std_devs[0]:.3f}, {std_devs[1]:.3f}, {std_devs[2]:.3f}]")


class MultiViewTriangulation:
    def __init__(self, cameras: List[Camera]):
        """
        Initialize the MultiView Geometry system with a list of cameras.

        Args:
            cameras: List of Camera objects
        """
        self.cameras = cameras
        self.camera_dict = {camera.camera_id: camera for camera in cameras}
        self.ceres = CeresOptimizer()

    def triangulate_midpoint(self,
                          point_2d_list: List[np.ndarray],
                          camera_id_list: List[int]) -> np.ndarray:
        """
        Triangulate a 3D point by finding the midpoint of the closest point between rays.

        Args:
            point_2d_list: List of 2D points in image coordinates (u, v)
            camera_id_list: List of camera IDs corresponding to each 2D point

        Returns:
            3D point in world coordinates (x, y, z)
        """
        if len(point_2d_list) != len(camera_id_list):
            raise ValueError("Number of 2D points must match number of camera IDs")

        if len(point_2d_list) < 2:
            raise ValueError("Need at least 2 views for triangulation")

        # Get rays from all cameras
        rays = []
        for i, camera_id in enumerate(camera_id_list):
            camera = self.camera_dict[camera_id]
            ray_origin, ray_direction = camera.get_ray(point_2d_list[i])
            rays.append((ray_origin, ray_direction))

        # Compute closest points between all pairs of rays
        closest_points = []
        distances = []  # Store distances between rays for uncertainty estimation

        for i in range(len(rays)):
            for j in range(i+1, len(rays)):
                origin1, dir1 = rays[i]
                origin2, dir2 = rays[j]

                # Compute closest points between two rays
                closest_point1, closest_point2 = self._closest_points_between_rays(origin1, dir1, origin2, dir2)

                # Use the midpoint of the two closest points
                midpoint = (closest_point1 + closest_point2) / 2
                closest_points.append(midpoint)

                # Store the distance between rays at their closest point
                # This will be used for uncertainty estimation
                distance = np.linalg.norm(closest_point1 - closest_point2)
                distances.append(distance)

        # Return the average of all closest points
        return np.mean(closest_points, axis=0)

    def _closest_points_between_rays(self,
                                  origin1: np.ndarray,
                                  dir1: np.ndarray,
                                  origin2: np.ndarray,
                                  dir2: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute the closest points between two rays.

        Args:
            origin1: Origin of the first ray
            dir1: Direction of the first ray
            origin2: Origin of the second ray
            dir2: Direction of the second ray

        Returns:
            closest_point1: Closest point on the first ray
            closest_point2: Closest point on the second ray
        """
        # Calculate the line parameters
        a = np.dot(dir1, dir1)
        b = np.dot(dir1, dir2)
        c = np.dot(dir2, dir2)
        d = np.dot(dir1, origin1 - origin2)
        e = np.dot(dir2, origin1 - origin2)

        # Check if rays are parallel
        denominator = a*c - b*b
        if abs(denominator) < 1e-10:
            # Rays are parallel, return a point on each ray
            return origin1, origin2

        # Compute the line parameters for the closest points
        t1 = (b*e - c*d) / denominator
        t2 = (a*e - b*d) / denominator

        # Compute the closest points
        closest_point1 = origin1 + t1 * dir1
        closest_point2 = origin2 + t2 * dir2

        return closest_point1, closest_point2

    def calculate_reprojection_error(self,
                                   point_3d: np.ndarray,
                                   point_2d_list: List[np.ndarray],
                                   camera_id_list: List[int]) -> float:
        """
        Calculate the reprojection error for a 3D point across multiple views.

        Args:
            point_3d: 3D point in world coordinates
            point_2d_list: List of observed 2D points in image coordinates
            camera_id_list: List of camera IDs corresponding to each 2D point

        Returns:
            Average reprojection error in pixels
        """
        total_error = 0.0

        for i, camera_id in enumerate(camera_id_list):
            camera = self.camera_dict[camera_id]
            projected_point = camera.project_point(point_3d)
            error = np.linalg.norm(projected_point - point_2d_list[i])
            total_error += error

        return total_error / len(point_2d_list)

    def estimate_uncertainty_from_ray_convergence(self,
                                     point_3d: np.ndarray,
                                     point_2d_list: List[np.ndarray],
                                     camera_id_list: List[int]) -> np.ndarray:
        """
        Estimate the uncertainty of a 3D point based on ray convergence geometry.

        This method estimates uncertainty by analyzing how the camera rays converge.
        The closer the rays come to each other, the lower the uncertainty.

        Args:
            point_3d: 3D point whose uncertainty we want to estimate
            point_2d_list: List of 2D points in image coordinates
            camera_id_list: List of camera IDs corresponding to each 2D point

        Returns:
            3x3 covariance matrix representing positional uncertainty
        """
        # Get rays from all cameras
        rays = []
        for i, camera_id in enumerate(camera_id_list):
            camera = self.camera_dict[camera_id]
            ray_origin, ray_direction = camera.get_ray(point_2d_list[i])
            rays.append((ray_origin, ray_direction))

        # Compute ray convergence metrics
        # How well do pairs of rays converge in each dimension?
        ray_convergence = np.zeros((3, 3))  # 3x3 matrix
        ray_count = 0

        for i in range(len(rays)):
            for j in range(i+1, len(rays)):
                origin1, dir1 = rays[i]
                origin2, dir2 = rays[j]

                # Compute closest points between two rays
                closest_point1, closest_point2 = self._closest_points_between_rays(origin1, dir1, origin2, dir2)

                # Distance between rays
                distance_vector = closest_point2 - closest_point1

                # Compute outer product to capture directional uncertainty
                # Rays that are nearly parallel have high uncertainty perpendicular to their direction
                outer_product = np.outer(distance_vector, distance_vector)

                # Add to the accumulator
                ray_convergence += outer_product
                ray_count += 1

        # Normalize by number of ray pairs
        if ray_count > 0:
            ray_convergence /= ray_count

        # Ensure the matrix is positive definite by adding a small constant to the diagonal
        ray_convergence += np.eye(3) * 1e-6

        # Scale based on number of views (more views = lower uncertainty)
        view_scale = max(1.0, len(rays)) ** -0.5
        ray_convergence *= view_scale

        # Scale based on reprojection error
        reprojection_error = self.calculate_reprojection_error(point_3d, point_2d_list, camera_id_list)
        error_scale = max(1.0, reprojection_error)
        ray_convergence *= error_scale

        return ray_convergence

    def optimize_3d_point(self,
                         initial_point: np.ndarray,
                         point_2d_list: List[np.ndarray],
                         camera_id_list: List[int]) -> UncertainPoint3D:
        """
        Optimize a 3D point to minimize reprojection error using Ceres.
        Also estimates the uncertainty of the optimized point.

        Args:
            initial_point: Initial estimate of the 3D point
            point_2d_list: List of observed 2D points in image coordinates
            camera_id_list: List of camera IDs corresponding to each 2D point

        Returns:
            UncertainPoint3D representing the optimized point with uncertainty
        """
        # Create a copy of the initial point for optimization
        point_3d = initial_point.copy().reshape(3, 1)

        # Create a Ceres problem
        problem = self.ceres.create_problem()

        # Add residual blocks for each observation
        for i, camera_id in enumerate(camera_id_list):
            camera = self.camera_dict[camera_id]
            cost_function = ReprojectionCostFunction(camera, point_2d_list[i])
            problem.add_residual_block(cost_function, None, point_3d)

        # Solve the optimization problem
        summary = self.ceres.solve(problem)

        # Get the optimized point
        optimized_point = point_3d.flatten()

        # Estimate covariance from the optimization
        try:
            # Get covariance from the optimization (if available)
            covariance = problem.get_covariance_matrix()

            # Extract the 3x3 portion for the 3D point
            if covariance.shape[0] > 3:
                covariance = covariance[:3, :3]

            # Ensure the covariance is positive definite
            eigenvals = np.linalg.eigvalsh(covariance)
            if np.any(eigenvals <= 0):
                # Fall back to ray convergence method
                covariance = self.estimate_uncertainty_from_ray_convergence(
                    optimized_point, point_2d_list, camera_id_list)
        except Exception:
            # If covariance estimation from optimization fails, use ray convergence
            covariance = self.estimate_uncertainty_from_ray_convergence(
                optimized_point, point_2d_list, camera_id_list)

        # Create an UncertainPoint3D
        return UncertainPoint3D(optimized_point, covariance)

    def find_correspondences(self,
                          bounding_boxes_per_camera: Dict[int, List[BoundingBox]],
                          max_reprojection_error: float = 10.0,
                          min_cameras_per_target: int = 2) -> List[List[BoundingBox]]:
        """
        Find correspondences between bounding boxes across different cameras.
        This method handles cases where:
        - Different cameras have different numbers of detections
        - Detections may be in any order (no ordering assumptions)
        - Some targets may not be visible in all cameras
        - Some cameras may have no detections
        - False positives may exist in any camera

        Args:
            bounding_boxes_per_camera: Dictionary mapping camera IDs to lists of bounding boxes
            max_reprojection_error: Maximum allowed reprojection error for a valid correspondence
            min_cameras_per_target: Minimum number of cameras needed to establish a target

        Returns:
            List of lists of bounding boxes, where each inner list contains corresponding bounding boxes
        """
        # Filter out cameras with no detections
        active_cameras = {cam_id: bbs for cam_id, bbs in bounding_boxes_per_camera.items() if bbs}

        # We need at least min_cameras_per_target cameras with detections
        if len(active_cameras) < min_cameras_per_target:
            return []

        # Start with all possible pairs of bounding boxes from different cameras
        camera_ids = sorted(active_cameras.keys())

        # Initialize potential targets with pairs from all camera combinations
        potential_targets = []

        # For all pairs of cameras
        for i in range(len(camera_ids) - 1):
            cam_i = camera_ids[i]
            for j in range(i + 1, len(camera_ids)):
                cam_j = camera_ids[j]

                # For all pairs of bounding boxes between these cameras
                for bb_i in active_cameras[cam_i]:
                    for bb_j in active_cameras[cam_j]:
                        # Try to triangulate a 3D point
                        try:
                            point_3d = self.triangulate_midpoint(
                                [bb_i.center, bb_j.center],
                                [cam_i, cam_j]
                            )

                            # Calculate reprojection error
                            error = self.calculate_reprojection_error(
                                point_3d,
                                [bb_i.center, bb_j.center],
                                [cam_i, cam_j]
                            )

                            # If error is acceptable, add to potential targets
                            if error < max_reprojection_error:
                                potential_targets.append({
                                    'point_3d': point_3d,
                                    'error': error,
                                    'bboxes': {cam_i: bb_i, cam_j: bb_j},
                                    'camera_count': 2  # Start with 2 cameras
                                })
                        except Exception as e:
                            # Silently continue if triangulation fails
                            pass

        # Early exit if no valid pairs found
        if not potential_targets:
            return []

        # Sort potential targets by 1) number of cameras and 2) reprojection error
        # This prioritizes targets seen by more cameras
        potential_targets.sort(key=lambda x: (-x['camera_count'], x['error']))

        # Greedily assign bounding boxes to targets
        used_bboxes = set()  # Set of (camera_id, detection_id) tuples for used bboxes
        final_targets = []

        for target in potential_targets:
            # Check if any of the bounding boxes in this target has already been used
            bboxes_used = False
            for cam_id, bbox in target['bboxes'].items():
                if (cam_id, bbox.detection_id) in used_bboxes:
                    bboxes_used = True
                    break

            if bboxes_used:
                continue

            # This is a good target, add its bounding boxes to used_bboxes
            for cam_id, bbox in target['bboxes'].items():
                used_bboxes.add((cam_id, bbox.detection_id))

            # Try to extend this target with bounding boxes from other cameras
            all_cam_bboxes = list(target['bboxes'].items())
            for cam_id in camera_ids:
                # Skip cameras that are already part of this target
                if cam_id in target['bboxes']:
                    continue

                # Skip cameras with no detections
                if cam_id not in active_cameras or not active_cameras[cam_id]:
                    continue

                # This camera is not yet part of this target
                best_bbox = None
                best_error = float('inf')
                best_point_3d = None

                # Try each bounding box from this camera
                for bbox in active_cameras[cam_id]:
                    # Skip bounding boxes that are already used
                    if (cam_id, bbox.detection_id) in used_bboxes:
                        continue

                    # Create a test list with this bbox added
                    test_points = [bb.center for _, bb in all_cam_bboxes] + [bbox.center]
                    test_cams = [c for c, _ in all_cam_bboxes] + [cam_id]

                    # Try triangulating with this additional bbox
                    try:
                        test_point_3d = self.triangulate_midpoint(test_points, test_cams)
                        test_error = self.calculate_reprojection_error(test_point_3d, test_points, test_cams)

                        # Accept only if error is below threshold and better than current best
                        if test_error < max_reprojection_error and test_error < best_error:
                            best_bbox = bbox
                            best_error = test_error
                            best_point_3d = test_point_3d
                    except Exception:
                        # Continue if triangulation fails
                        pass

                # If we found a good match, add it to this target
                if best_bbox is not None:
                    target['bboxes'][cam_id] = best_bbox
                    target['point_3d'] = best_point_3d
                    target['error'] = best_error
                    target['camera_count'] += 1
                    used_bboxes.add((cam_id, best_bbox.detection_id))

            # Only add targets that are observed by enough cameras
            if target['camera_count'] >= min_cameras_per_target:
                final_targets.append(list(target['bboxes'].values()))

        return final_targets

    def estimate_detection_uncertainty(self,
                                 bounding_box: BoundingBox,
                                 pixel_std_dev: float = 2.0) -> np.ndarray:
        """
        Estimate the uncertainty of a bounding box detection in image space.

        Args:
            bounding_box: The bounding box detection
            pixel_std_dev: Standard deviation in pixels (default is 2.0)

        Returns:
            2x2 covariance matrix representing uncertainty in image coordinates
        """
        # A simple model: larger bounding boxes have higher uncertainty
        # We'll scale the base uncertainty by the size of the bounding box
        size_factor = np.sqrt(bounding_box.width * bounding_box.height) / 100.0

        # Base covariance: diagonal matrix with pixel_std_dev^2 on the diagonal
        base_cov = np.eye(2) * (pixel_std_dev ** 2)

        # Scale by size factor (minimum of 1.0)
        return base_cov * max(1.0, size_factor)

    def estimate_uncertainty_from_monte_carlo(self,
                                           initial_point: np.ndarray,
                                           bounding_boxes: List[BoundingBox],
                                           num_samples: int = 100) -> np.ndarray:
        """
        Estimate uncertainty using Monte Carlo sampling of detection noise.

        Args:
            initial_point: Initial estimate of the 3D point
            bounding_boxes: List of bounding box detections
            num_samples: Number of Monte Carlo samples

        Returns:
            3x3 covariance matrix representing positional uncertainty
        """
        # Extract centers and camera IDs
        centers = [bb.center for bb in bounding_boxes]
        camera_ids = [bb.camera_id for bb in bounding_boxes]

        # Generate perturbed versions of the detections
        samples = []

        for _ in range(num_samples):
            perturbed_centers = []

            # Add noise to each detection based on its estimated uncertainty
            for i, bb in enumerate(bounding_boxes):
                # Estimate the uncertainty of this detection
                cov = self.estimate_detection_uncertainty(bb)

                # Sample noise from a multivariate normal distribution
                noise = np.random.multivariate_normal([0, 0], cov)

                # Add noise to the center
                perturbed_center = centers[i] + noise
                perturbed_centers.append(perturbed_center)

            # Triangulate with the perturbed centers
            try:
                # Use midpoint method for speed
                sample_point = self.triangulate_midpoint(perturbed_centers, camera_ids)
                samples.append(sample_point)
            except Exception:
                # Skip this sample if triangulation fails
                pass

        # Compute the sample covariance matrix
        if len(samples) > 1:
            samples = np.array(samples)
            covariance = np.cov(samples, rowvar=False)

            # Ensure the covariance is positive definite
            eigenvals = np.linalg.eigvalsh(covariance)
            if np.any(eigenvals <= 0):
                # Add a small constant to the diagonal
                covariance += np.eye(3) * 1e-6

            return covariance
        else:
            # Fallback if Monte Carlo sampling failed
            return np.eye(3) * 0.1  # Default uncertainty

    def localize_from_bounding_boxes(self,
                                   bounding_boxes: List[BoundingBox]) -> UncertainPoint3D:
        """
        Localize a target in 3D space from multiple bounding box observations.
        Returns the target position with uncertainty estimate.

        Args:
            bounding_boxes: List of BoundingBox objects from different cameras

        Returns:
            UncertainPoint3D representing the target position with uncertainty
        """
        # Extract bounding box centers and camera IDs
        centers = [bb.center for bb in bounding_boxes]
        camera_ids = [bb.camera_id for bb in bounding_boxes]

        # Get initial guess using midpoint method
        initial_guess = self.triangulate_midpoint(centers, camera_ids)

        # Estimate uncertainty using multiple methods for robustness

        # 1. Monte Carlo sampling
        mc_covariance = self.estimate_uncertainty_from_monte_carlo(initial_guess, bounding_boxes)

        # 2. Optimize using reprojection error and get uncertainty
        uncertain_point = self.optimize_3d_point(initial_guess, centers, camera_ids)

        # Combine the covariances from different methods
        # We use the optimization-based covariance as primary, but add information from Monte Carlo
        combined_covariance = (uncertain_point.covariance + mc_covariance) / 2

        # Create a new UncertainPoint3D with the combined covariance
        return UncertainPoint3D(uncertain_point.position, combined_covariance)

    def localize_multiple_targets(self,
                               bounding_boxes_per_camera: Dict[int, List[BoundingBox]],
                               max_reprojection_error: float = 10.0,
                               min_cameras_per_target: int = 2) -> List[Tuple[UncertainPoint3D, List[BoundingBox]]]:
        """
        Localize multiple targets in 3D space from multiple bounding box observations.
        Returns target positions with uncertainty estimates.

        Args:
            bounding_boxes_per_camera: Dictionary mapping camera IDs to lists of bounding boxes
            max_reprojection_error: Maximum allowed reprojection error for a valid correspondence
            min_cameras_per_target: Minimum number of cameras needed to establish a target

        Returns:
            List of tuples, each containing:
              - UncertainPoint3D representing the target position with uncertainty
              - List of bounding boxes used for this target (for debugging/visualization)
        """
        # Find correspondences between bounding boxes
        target_groups = self.find_correspondences(
            bounding_boxes_per_camera,
            max_reprojection_error,
            min_cameras_per_target
        )

        # Localize each target and return positions with their corresponding bounding boxes
        return [(self.localize_from_bounding_boxes(group), group) for group in target_groups]

    def calculate_uncertainty_confidence_intervals(self,
                                               uncertain_point: UncertainPoint3D,
                                               confidence: float = 0.95) -> Dict:
        """
        Calculate confidence intervals for the uncertain point's position.

        Args:
            uncertain_point: UncertainPoint3D to analyze
            confidence: Confidence level (e.g., 0.95 for 95% confidence)

        Returns:
            Dictionary with confidence interval information
        """
        # Get the confidence ellipsoid
        radii, axes = uncertain_point.confidence_ellipsoid(confidence)

        # Calculate confidence intervals for each axis
        intervals = {}

        # Get standard deviations
        std_devs = np.sqrt(np.diag(uncertain_point.covariance))

        # For a normal distribution, 95% confidence interval is approximately ±1.96 standard deviations
        confidence_factors = {
            0.68: 1.0,   # ~1 sigma
            0.95: 1.96,  # ~2 sigma
            0.99: 2.58   # ~3 sigma
        }

        factor = confidence_factors.get(confidence, 1.96)  # Default to 95% confidence

        # Calculate intervals
        intervals['x'] = (
            uncertain_point.position[0] - factor * std_devs[0],
            uncertain_point.position[0] + factor * std_devs[0]
        )
        intervals['y'] = (
            uncertain_point.position[1] - factor * std_devs[1],
            uncertain_point.position[1] + factor * std_devs[1]
        )
        intervals['z'] = (
            uncertain_point.position[2] - factor * std_devs[2],
            uncertain_point.position[2] + factor * std_devs[2]
        )

        # Overall position uncertainty (volume of the ellipsoid)
        volume = (4/3) * np.pi * np.prod(radii)

        # Maximum uncertainty direction and magnitude
        max_direction_idx = np.argmax(radii)
        max_direction = axes[:, max_direction_idx]
        max_uncertainty = radii[max_direction_idx]

        return {
            'confidence_level': confidence,
            'intervals': intervals,
            'ellipsoid_radii': radii,
            'ellipsoid_axes': axes,
            'uncertainty_volume': volume,
            'max_uncertainty_direction': max_direction,
            'max_uncertainty_magnitude': max_uncertainty
        }


def load_camera_parameters(image_width: int,
                         image_height: int,
                         intrinsic_params: Dict,
                         extrinsic_params: Dict,
                         camera_id: int) -> Camera:
    """
    Load camera parameters from dictionaries.

    Args:
        image_width: Width of the image in pixels
        image_height: Height of the image in pixels
        intrinsic_params: Dictionary with intrinsic parameters (fx, fy, cx, cy)
        extrinsic_params: Dictionary with extrinsic parameters (rotation matrix and translation vector)
        camera_id: Unique identifier for the camera

    Returns:
        Camera object
    """
    # Construct camera intrinsic matrix
    fx = intrinsic_params.get('fx')
    fy = intrinsic_params.get('fy')
    cx = intrinsic_params.get('cx')
    cy = intrinsic_params.get('cy')

    K = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ])

    # Construct camera extrinsic matrix
    R = extrinsic_params.get('rotation_matrix')
    t = extrinsic_params.get('translation_vector')

    # Create 4x4 extrinsic matrix [R|t; 0 0 0 1]
    extrinsic = np.zeros((4, 4))
    extrinsic[:3, :3] = R
    extrinsic[:3, 3] = t
    extrinsic[3, 3] = 1

    # Create camera object
    camera = Camera(camera_id, image_width, image_height, K, extrinsic)

    return camera


def create_bounding_box(center_u: float,
                      center_v: float,
                      width: float,
                      height: float,
                      camera_id: int,
                      detection_id: int = -1) -> BoundingBox:
    """
    Create a bounding box object.

    Args:
        center_u: U-coordinate of the bounding box center
        center_v: V-coordinate of the bounding box center
        width: Width of the bounding box in pixels
        height: Height of the bounding box in pixels
        camera_id: ID of the camera that observed this bounding box
        detection_id: ID of the detection within a camera

    Returns:
        BoundingBox object
    """
    center = np.array([center_u, center_v])
    return BoundingBox(center, width, height, camera_id, detection_id)


def demo():
    """
    Demonstration of using the MultiViewGeometry library with multiple targets
    and varying camera detections, including uncertainty estimation.
    """
    # Define camera parameters
    image_width = 1920
    image_height = 1080

    # Camera 1
    intrinsic1 = {
        'fx': 1000.0,
        'fy': 1000.0,
        'cx': image_width / 2,
        'cy': image_height / 2
    }

    extrinsic1 = {
        'rotation_matrix': np.eye(3),
        'translation_vector': np.array([0, 0, 0])
    }

    # Camera 2
    intrinsic2 = {
        'fx': 1000.0,
        'fy': 1000.0,
        'cx': image_width / 2,
        'cy': image_height / 2
    }

    extrinsic2 = {
        'rotation_matrix': np.array([
            [0, 0, 1],
            [0, 1, 0],
            [-1, 0, 0]
        ]),
        'translation_vector': np.array([5, 0, 0])
    }

    # Camera 3
    intrinsic3 = {
        'fx': 1000.0,
        'fy': 1000.0,
        'cx': image_width / 2,
        'cy': image_height / 2
    }

    extrinsic3 = {
        'rotation_matrix': np.array([
            [1, 0, 0],
            [0, 0, -1],
            [0, 1, 0]
        ]),
        'translation_vector': np.array([0, 5, 0])
    }

    # Create camera objects
    camera1 = load_camera_parameters(image_width, image_height, intrinsic1, extrinsic1, 1)
    camera2 = load_camera_parameters(image_width, image_height, intrinsic2, extrinsic2, 2)
    camera3 = load_camera_parameters(image_width, image_height, intrinsic3, extrinsic3, 3)

    cameras = [camera1, camera2, camera3]

    # Create MultiViewGeometry system
    mvg = MultiViewTriangulation(cameras)

    # Create multiple targets in 3D space
    target1_3d = np.array([2.0, 1.0, 10.0])
    target2_3d = np.array([3.0, 2.0, 8.0])
    target3_3d = np.array([-1.0, 3.0, 12.0])

    # Project targets to each camera
    target1_2d_cam1 = camera1.project_point(target1_3d)
    target1_2d_cam2 = camera2.project_point(target1_3d)
    target1_2d_cam3 = camera3.project_point(target1_3d)

    target2_2d_cam1 = camera1.project_point(target2_3d)
    target2_2d_cam2 = camera2.project_point(target2_3d)
    target2_2d_cam3 = camera3.project_point(target2_3d)

    target3_2d_cam1 = camera1.project_point(target3_3d)
    target3_2d_cam2 = camera2.project_point(target3_3d)
    # Let's say target3 is not visible from camera3

    # Create bounding boxes with fixed size
    bb_width = 50
    bb_height = 100

    # Create bounding boxes for camera 1
    bb1_1 = create_bounding_box(target1_2d_cam1[0], target1_2d_cam1[1], bb_width, bb_height, 1, 0)
    bb1_2 = create_bounding_box(target2_2d_cam1[0], target2_2d_cam1[1], bb_width, bb_height, 1, 1)
    bb1_3 = create_bounding_box(target3_2d_cam1[0], target3_2d_cam1[1], bb_width, bb_height, 1, 2)

    # Create bounding boxes for camera 2
    bb2_1 = create_bounding_box(target1_2d_cam2[0], target1_2d_cam2[1], bb_width, bb_height, 2, 0)
    bb2_2 = create_bounding_box(target2_2d_cam2[0], target2_2d_cam2[1], bb_width, bb_height, 2, 1)
    bb2_3 = create_bounding_box(target3_2d_cam2[0], target3_2d_cam2[1], bb_width, bb_height, 2, 2)

    # Create bounding boxes for camera 3
    bb3_1 = create_bounding_box(target1_2d_cam3[0], target1_2d_cam3[1], bb_width, bb_height, 3, 0)
    bb3_2 = create_bounding_box(target2_2d_cam3[0], target2_2d_cam3[1], bb_width, bb_height, 3, 1)
    # No bounding box for target3 in camera3

    # Also add a false detection in camera 3
    bb3_false = create_bounding_box(800, 600, bb_width, bb_height, 3, 2)

    # Create test case with differently ordered and incomplete bounding boxes
    print("=== DEMO 1: Standard case with all detections and uncertainty estimation ===")
    bounding_boxes_per_camera = {
        1: [bb1_1, bb1_2, bb1_3],
        2: [bb2_1, bb2_2, bb2_3],
        3: [bb3_1, bb3_2, bb3_false]
    }

    # Localize multiple targets
    results = mvg.localize_multiple_targets(bounding_boxes_per_camera)
    uncertain_points = [uncertain_point for uncertain_point, _ in results]

    print(f"Ground truth 3D positions:")
    print(f"Target 1: {target1_3d}")
    print(f"Target 2: {target2_3d}")
    print(f"Target 3: {target3_3d}")

    print(f"\nEstimated 3D positions with uncertainty ({len(uncertain_points)} targets found):")
    for i, (uncertain_point, bboxes) in enumerate(results):
        camera_ids = [bb.camera_id for bb in bboxes]
        print(f"Target {i+1}:")
        print(f"  {uncertain_point}")
        print(f"  Seen in cameras: {camera_ids}")

        # Calculate and display uncertainty metrics
        uncertainty_info = mvg.calculate_uncertainty_confidence_intervals(uncertain_point)

        # Display the 95% confidence intervals
        intervals = uncertainty_info['intervals']
        print(f"  95% Confidence Intervals:")
        print(f"    X: [{intervals['x'][0]:.3f}, {intervals['x'][1]:.3f}]")
        print(f"    Y: [{intervals['y'][0]:.3f}, {intervals['y'][1]:.3f}]")
        print(f"    Z: [{intervals['z'][0]:.3f}, {intervals['z'][1]:.3f}]")

        # Display the direction of maximum uncertainty
        max_dir = uncertainty_info['max_uncertainty_direction']
        max_mag = uncertainty_info['max_uncertainty_magnitude']
        print(f"  Maximum uncertainty: {max_mag:.3f} units along [{max_dir[0]:.3f}, {max_dir[1]:.3f}, {max_dir[2]:.3f}]")

        # Calculate error if ground truth is available (for demo purposes only)
        if i < len([target1_3d, target2_3d, target3_3d]):
            gt = [target1_3d, target2_3d, target3_3d][i]
            error = np.linalg.norm(uncertain_point.position - gt)

            # Check if the ground truth is within the confidence ellipsoid
            m_distance = uncertain_point.mahalanobis_distance(gt)
            in_ellipsoid = m_distance <= np.sqrt(7.81)  # Chi-square value for 95% confidence in 3D

            print(f"  Error: {error:.3f} units")
            print(f"  Ground truth within 95% confidence ellipsoid: {in_ellipsoid}")

    # ======================================================
    # Now test with varying numbers and order of detections
    print("\n=== DEMO 2: Different camera configurations and uncertainty ===")

    # Test different camera configurations and their impact on uncertainty

    # Configuration 1: Two cameras at 90 degrees
    print("\n-- Configuration 1: Two orthogonal cameras --")
    bounding_boxes_per_camera_conf1 = {
        1: [bb1_1],
        2: [bb2_1]
    }

    results_conf1 = mvg.localize_multiple_targets(bounding_boxes_per_camera_conf1)
    if results_conf1:
        uncertain_point_conf1, _ = results_conf1[0]
        print(f"Target position: {uncertain_point_conf1.position}")
        std_devs_conf1 = np.sqrt(np.diag(uncertain_point_conf1.covariance))
        print(f"Standard deviations: {std_devs_conf1}")

    # Configuration 2: Three cameras in good configuration
    print("\n-- Configuration 2: Three cameras in good configuration --")
    bounding_boxes_per_camera_conf2 = {
        1: [bb1_1],
        2: [bb2_1],
        3: [bb3_1]
    }

    results_conf2 = mvg.localize_multiple_targets(bounding_boxes_per_camera_conf2)
    if results_conf2:
        uncertain_point_conf2, _ = results_conf2[0]
        print(f"Target position: {uncertain_point_conf2.position}")
        std_devs_conf2 = np.sqrt(np.diag(uncertain_point_conf2.covariance))
        print(f"Standard deviations: {std_devs_conf2}")

    # Configuration 3: Two cameras with near-parallel view directions (higher uncertainty)
    print("\n-- Configuration 3: Two near-parallel cameras (higher uncertainty) --")

    # Create a camera that's almost parallel to camera 1
    intrinsic4 = {
        'fx': 1000.0,
        'fy': 1000.0,
        'cx': image_width / 2,
        'cy': image_height / 2
    }

    # Only slightly different from camera 1
    extrinsic4 = {
        'rotation_matrix': np.array([
            [0.99, 0, 0.01],
            [0, 1, 0],
            [-0.01, 0, 0.99]
        ]),
        'translation_vector': np.array([0.5, 0, 0])
    }

    camera4 = load_camera_parameters(image_width, image_height, intrinsic4, extrinsic4, 4)

    # Project target to this camera
    target1_2d_cam4 = camera4.project_point(target1_3d)
    bb4_1 = create_bounding_box(target1_2d_cam4[0], target1_2d_cam4[1], bb_width, bb_height, 4, 0)

    # Create a temporary MVG with the parallel cameras
    temp_cameras = [camera1, camera4]
    temp_mvg = MultiViewTriangulation(temp_cameras)

    bounding_boxes_per_camera_conf3 = {
        1: [bb1_1],
        4: [bb4_1]
    }

    results_conf3 = temp_mvg.localize_multiple_targets(bounding_boxes_per_camera_conf3)
    if results_conf3:
        uncertain_point_conf3, _ = results_conf3[0]
        print(f"Target position: {uncertain_point_conf3.position}")
        std_devs_conf3 = np.sqrt(np.diag(uncertain_point_conf3.covariance))
        print(f"Standard deviations: {std_devs_conf3}")

        # Compare uncertainty volumes
        if results_conf1 and results_conf2:
            print("\n-- Uncertainty Comparison --")

            # Volume comparison
            vol1 = np.sqrt(np.linalg.det(uncertain_point_conf1.covariance))
            vol2 = np.sqrt(np.linalg.det(uncertain_point_conf2.covariance))
            vol3 = np.sqrt(np.linalg.det(uncertain_point_conf3.covariance))

            print(f"Configuration 1 (90° cameras) uncertainty volume: {vol1:.6f}")
            print(f"Configuration 2 (3 cameras) uncertainty volume: {vol2:.6f}")
            print(f"Configuration 3 (parallel cameras) uncertainty volume: {vol3:.6f}")

            # Relative improvement
            print(f"Adding a third camera reduces uncertainty by: {(vol1-vol2)/vol1*100:.2f}%")
            print(f"Parallel vs. orthogonal cameras difference: {(vol3-vol1)/vol1*100:.2f}%")


if __name__ == "__main__":
    demo()