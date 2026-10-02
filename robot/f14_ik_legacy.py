import numpy as np
import pinocchio as pin


LEFT_JOINT_NAMES = [
    "left_dof1_joint",
    "left_dof2_joint",
    "left_dof3_joint",
    "left_dof4_joint",
    "left_dof5_joint",
    "left_dof6_joint",
    "left_dof7_joint",
]

RIGHT_JOINT_NAMES = [
    "right_dof1_joint",
    "right_dof2_joint",
    "right_dof3_joint",
    "right_dof4_joint",
    "right_dof5_joint",
    "right_dof6_joint",
    "right_dof7_joint",
]


class F14IK:

    def __init__(self, urdf_path):

        self.model = pin.buildModelFromUrdf(
            urdf_path
        )

        self.data = self.model.createData()

        print("Pinocchio model loaded")
        print("nq =", self.model.nq)
        print("nv =", self.model.nv)

        # End-effector frames
        self.left_ee_frame = self.model.getFrameId(
            "left_dof7_link"
        )

        self.right_ee_frame = self.model.getFrameId(
            "right_dof7_link"
        )

        if self.left_ee_frame >= len(self.model.frames):
            raise ValueError(
                "left_dof7_link frame not found"
            )

        if self.right_ee_frame >= len(self.model.frames):
            raise ValueError(
                "right_dof7_link frame not found"
            )

        self.arm_q_indices = []
        self.arm_v_indices = []

        for name in LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES:

            joint_id = self.model.getJointId(name)

            if joint_id == 0:
                raise ValueError(
                    f"Joint not found: {name}"
                )

            joint = self.model.joints[joint_id]

            self.arm_q_indices.append(
                joint.idx_q
            )

            self.arm_v_indices.append(
                joint.idx_v
            )

        self.arm_q_indices = np.array(
            self.arm_q_indices,
            dtype=int
        )

        self.arm_v_indices = np.array(
            self.arm_v_indices,
            dtype=int
        )




        print("arm q indices:")
        print(self.arm_q_indices)
        print("arm v indices:")
        print(self.arm_v_indices)
        print(
            "Left EE frame:",
            self.model.frames[
                self.left_ee_frame
            ].name
        )

        print(
            "Right EE frame:",
            self.model.frames[
                self.right_ee_frame
            ].name
        )
        
    def arm_to_full_q(self, arm_q):

        arm_q = np.asarray(
            arm_q,
            dtype=float
        )

        if arm_q.shape != (14,):
            raise ValueError(
                f"Expected (14,), got {arm_q.shape}"
            )

        q = pin.neutral(
            self.model
        )

        q[self.arm_q_indices] = arm_q

        return q
        
    def full_to_arm_q(self, q):
        return q[
            self.arm_q_indices
        ].copy()
        
        
        
    def forward_kinematics(self, arm_q):
    
        q = self.arm_to_full_q(
            arm_q
        )
    
        pin.forwardKinematics(
            self.model,
            self.data,
            q
        )
    
        pin.updateFramePlacements(
            self.model,
            self.data
        )
    
        left_pose = self.data.oMf[
            self.left_ee_frame
        ].copy()
    
        right_pose = self.data.oMf[
            self.right_ee_frame
        ].copy()

        return left_pose, right_pose
   
    def solve(
        self,
        target_left,
        target_right,
        arm_q_init,
        max_iter=200,
        eps=1e-4,
        dt=0.2,
        damping=1e-5,
        verbose=False,
    ):
        q = self.arm_to_full_q(
            arm_q_init
        )

        for i in range(max_iter):

            # ---------------------
            # FK
            # ---------------------

            pin.forwardKinematics(
                self.model,
                self.data,
                q
            )

            pin.updateFramePlacements(
                self.model,
                self.data
            )


            current_left = self.data.oMf[
                self.left_ee_frame
            ]

            current_right = self.data.oMf[
                self.right_ee_frame
            ]


            # ---------------------
            # SE(3) error
            # ---------------------

            left_error = pin.log6(
                current_left.inverse()
                * target_left
            ).vector

            right_error = pin.log6(
                current_right.inverse()
                * target_right
            ).vector


            error = np.concatenate([
                left_error,
                right_error
            ])


            error_norm = np.linalg.norm(
                error
            )


            if error_norm < eps:
                if verbose:
                    print(
                        f"IK converged at "
                        f"iteration {i},"
                        f"error = {error_norm:.6e}"
                    )

                return (
                    self.full_to_arm_q(q),
                    True
                )


            # ---------------------
            # Jacobian
            # ---------------------

            J_left = pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                self.left_ee_frame,
                pin.ReferenceFrame.LOCAL
            )

            J_right = pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                self.right_ee_frame,
                pin.ReferenceFrame.LOCAL
            )


            J = np.vstack([
                J_left,
                J_right
            ])


            # ---------------------
            # damped least squares
            # ---------------------

            JJt = J @ J.T

            v = J.T @ np.linalg.solve(
                JJt
                + damping
                * np.eye(JJt.shape[0]),
                error
            )


            q = pin.integrate(
                self.model,
                q,
                v * dt
            )


            # ---------------------
            # joint limits
            # ---------------------

            q = np.minimum(
                np.maximum(
                    q,
                    self.model.lowerPositionLimit
                ),
                self.model.upperPositionLimit
            )


        print(
            "IK failed to fully converge. "
            f"Final error = {error_norm:.6e}"
        )

        return (
            self.full_to_arm_q(q),
            False
        )
