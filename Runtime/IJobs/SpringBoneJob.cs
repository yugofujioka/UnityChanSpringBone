using Unity.Collections;
using Unity.Jobs;
using UnityEngine;


namespace Unity.Animations.SpringBones.JobExtend {
	/// <summary>
	/// 全体設定
	/// </summary>
	public struct SpringBoneSettings {
		public float deltaTime;
		public float dynamicRatio;
		public Vector3 gravity;
		public float bounce;
		public float friction;
		public bool enableAngleLimits;
		public bool enableCollision;
		public bool enableLengthLimits;
		public bool collideWithGround;
		public float groundHeight;
	}

	/// <summary>
	/// ボーン毎の設定値（固定）
	/// </summary>
	[System.Serializable]
	public struct SpringBoneProperties {
		public float stiffnessForce;
		public float dragForce;
		public Vector3 springForce;
		public float windInfluence;
		public float angularStiffness;
		public Jobs.AngleLimitComponent yAngleLimits;
		public Jobs.AngleLimitComponent zAngleLimits;
		public float radius;
		public float springLength;
		public Vector3 boneAxis;
        public int collisionMask;

		public int lengthLimitIndex;
		public int lengthLimitLength;
		
		public int parentIndex;
		public Vector3 localPosition;
		public Quaternion initialLocalRotation;
	}

	/// <summary>
	/// ボーン毎のJob内で書き換える値
	/// </summary>
	[System.Serializable]
	public struct SpringBoneComponent {
		public Vector3 currentTipPosition;
		public Vector3 previousTipPosition;
		public Quaternion localRotation;
		public Vector3 position;
		public Quaternion rotation;
		public Vector3 parentPosition;
		public Quaternion parentRotation;
		public Matrix4x4 pivotLocalToGlobalMat;
	}

	[System.Serializable]
	public struct LengthLimitComponent {
		public Vector3 position;
		public float target;
	}

	//[Burst.BurstCompile]
	public partial struct SpringBoneJob : IJob {
		[ReadOnly]
		public SpringBoneSettings settings;
		[ReadOnly]
		public NativeArray<SpringBoneProperties> properties;
		[ReadOnly]
		public NativeArray<LengthLimitComponent> lengthLimits;
		[ReadOnly]
		public NativeArray<Jobs.SpringColliderComponent> colliders;
		[ReadOnly]
		public NativeArray<Jobs.SpringColliderTransform> colliderTransforms;

		public NativeArray<SpringBoneComponent> components;


		/// <summary>
		/// ジョブ実行
		/// </summary>
		void IJob.Execute() {
			var length = this.components.Length;
			for (int index = 0; index < length; ++index) {
				var bone = this.components[index];
				var prop = this.properties[index];

				if (prop.parentIndex >= 0) {
					// 親ノードがSpringBoneなら演算結果を反映する
					var parentBone = this.components[prop.parentIndex];
					bone.parentPosition = parentBone.position;
					bone.parentRotation = parentBone.rotation;
				}
				bone.position = bone.parentPosition + bone.parentRotation * prop.localPosition;
				bone.rotation = bone.parentRotation * bone.localRotation;

				var baseWorldRotation = bone.parentRotation * prop.initialLocalRotation;
				this.UpdateSpring(ref bone, in prop, in baseWorldRotation);
				this.ResolveCollisionsAndConstraints(ref bone, in prop);

				this.UpdateRotation(ref bone, in prop, in baseWorldRotation);
				components[index] = bone;
			}
		}

		private void UpdateSpring(ref SpringBoneComponent bone, in SpringBoneProperties prop, in Quaternion baseWorldRotation) {
			var orientedInitialPosition = bone.position + baseWorldRotation * prop.boneAxis * prop.springLength;

			// Hooke's law: force to push us to equilibrium
			var force = prop.stiffnessForce * (orientedInitialPosition - bone.currentTipPosition);
			force += prop.springForce + settings.gravity; // TODO: externalForce
			var sqrDt = settings.deltaTime * settings.deltaTime;
			force *= 0.5f * sqrDt;

			var temp = bone.currentTipPosition;
			force += (1f - prop.dragForce) * (bone.currentTipPosition - bone.previousTipPosition);
			bone.currentTipPosition += force;
			bone.previousTipPosition = temp;

			// Inlined because FixBoneLength is slow
			var headPosition = bone.position;
			var headToTail = bone.currentTipPosition - headPosition;
			var magnitude = Vector3.Magnitude(headToTail);

			const float MagnitudeThreshold = 0.001f;
			if (magnitude <= MagnitudeThreshold) {
				// was originally this
				//headToTail = transform.TransformDirection(boneAxis)
				Matrix4x4 mat = Matrix4x4.TRS(bone.position, bone.rotation, Vector3.one);
				headToTail = mat.MultiplyVector(headToTail);
			} else {
				headToTail /= magnitude;
			}

			bone.currentTipPosition = headPosition + prop.springLength * headToTail;
		}

		private void ResolveCollisionsAndConstraints(ref SpringBoneComponent bone, in SpringBoneProperties prop) {
			if (this.settings.enableLengthLimits)
				this.ApplyLengthLimits(ref bone, in prop, this.settings.deltaTime);

			var hadCollision = false;

			if (this.settings.collideWithGround)
				hadCollision = ResolveGroundCollision(ref bone, in prop, this.settings.groundHeight);

			if (this.settings.enableCollision && !hadCollision)
				this.ResolveCollisions(ref bone, in prop);

			if (this.settings.enableAngleLimits)
				ApplyAngleLimits(ref bone, in prop, this.settings.deltaTime);
		}

		// Returns the new tip position
		private void ApplyLengthLimits(ref SpringBoneComponent bone, in SpringBoneProperties prop, float deltaTime) {
			var targetCount = prop.lengthLimitLength;
			if (targetCount == 0)
				return;

			const float SpringConstant = 0.5f;
			var accelerationMultiplier = SpringConstant * deltaTime * deltaTime;
			var movement = Vector3.zero;
			var start = prop.lengthLimitIndex;
			var length = start + prop.lengthLimitLength;
			for (int i = start; i < length; ++i) {
				var limit = this.lengthLimits[i];
				var lengthToLimitTarget = limit.target;
				var currentToTarget = bone.currentTipPosition - limit.position;
				//var currentDistanceSquared = Vector3.SqrMagnitude(currentToTarget);

				// Hooke's Law
				//var currentDistance = Mathf.Sqrt(currentDistanceSquared);
				var currentDistance = Vector3.Magnitude(currentToTarget);
				var distanceFromEquilibrium = currentDistance - lengthToLimitTarget;
				//movement -= accelerationMultiplier * distanceFromEquilibrium * Vector3.Normalize(currentToTarget);
				movement -= accelerationMultiplier * distanceFromEquilibrium * (currentToTarget / currentDistance);
			}

			bone.currentTipPosition += movement;
		}

		private static bool ResolveGroundCollision(ref SpringBoneComponent bone, in SpringBoneProperties prop, float groundHeight) {
			// Todo: this assumes a flat ground parallel to the xz plane
			var worldHeadPosition = bone.position;
			var worldTailPosition = bone.currentTipPosition;
			// NOTE: スケールが反映されないのだからスカラー値が欲しいなら行列演算する意味はないのでは…？
			var worldRadius = prop.radius;//transform.TransformDirection(prop.radius, 0f, 0f).magnitude;
			var worldLength = Vector3.Magnitude(bone.currentTipPosition - worldHeadPosition);
			worldHeadPosition.y -= groundHeight;
			worldTailPosition.y -= groundHeight;

			var collidingWithGround = Jobs.SpringCollisionResolver.ResolvePanelOnAxis(
				worldHeadPosition, ref worldTailPosition, worldLength, worldRadius, Jobs.SpringCollisionResolver.Axis.Y);

			if (collidingWithGround) {
				worldTailPosition.y += groundHeight;
				bone.currentTipPosition = FixBoneLength(ref bone, in prop, in worldTailPosition);
				// Todo: bounce, friction
				bone.previousTipPosition = bone.currentTipPosition;
			}

			return collidingWithGround;
		}

		private static Vector3 FixBoneLength(ref SpringBoneComponent bone, in SpringBoneProperties prop, in Vector3 tailPosition) {
			var minLength = 0.5f * prop.springLength;
			var maxLength = prop.springLength;
			var headPosition = bone.position;
			var headToTail = tailPosition - headPosition;
			var magnitude = headToTail.magnitude;

			const float MagnitudeThreshold = 0.001f;
			if (magnitude <= MagnitudeThreshold) {
				Matrix4x4 mat = Matrix4x4.TRS(bone.position, bone.rotation, Vector3.one);
				return headPosition + mat.MultiplyVector(prop.boneAxis) * minLength;
			}

			var newMagnitude = (magnitude < minLength) ? minLength : magnitude;
			newMagnitude = (newMagnitude > maxLength) ? maxLength : newMagnitude;
			return headPosition + (newMagnitude / magnitude) * headToTail;
		}

		private bool ResolveCollisions(ref SpringBoneComponent bone, in SpringBoneProperties prop) {
			var desiredPosition = bone.currentTipPosition;
			var headPosition = bone.position;

			//            var scaledRadius = transform.TransformDirection(radius, 0f, 0f).magnitude;
			// var scaleMagnitude = new Vector3(prop.radius, 0f, 0f).magnitude;
			var hitNormal = new Vector3(0f, 0f, 1f);

			var hadCollision = false;

			var length = this.colliders.Length;
			for (var i = 0; i < length; ++i) {
				var collider = this.colliders[i];
				var colliderTransform = this.colliderTransforms[i];

				// comment out for testing
				if ((prop.collisionMask & (1 << collider.layer)) == 0) {
					continue;
				}

				switch (collider.type) {
					case Jobs.ColliderType.Capsule:
						hadCollision |= Jobs.SpringCollisionResolver.ResolveCapsule(
							collider, colliderTransform,
							headPosition, ref bone.currentTipPosition, ref hitNormal,
							prop.radius);
						break;
					case Jobs.ColliderType.Sphere:
						hadCollision |= Jobs.SpringCollisionResolver.ResolveSphere(
							collider, colliderTransform,
							headPosition, ref bone.currentTipPosition, ref hitNormal, prop.radius);
						break;
					case Jobs.ColliderType.Panel:
						hadCollision |= Jobs.SpringCollisionResolver.ResolvePanel(
							collider, colliderTransform,
							headPosition, ref bone.currentTipPosition, ref hitNormal, prop.springLength,
							prop.radius);
						break;
				}
			}

			if (hadCollision) {
				var incidentVector = desiredPosition - bone.previousTipPosition;
				var reflectedVector = Vector3.Reflect(incidentVector, hitNormal);

				// friction
				var upwardComponent = Vector3.Dot(reflectedVector, hitNormal) * hitNormal;
				var lateralComponent = reflectedVector - upwardComponent;

				var bounceVelocity = this.settings.bounce * upwardComponent + (1f - this.settings.friction) * lateralComponent;
				const float BounceThreshold = 0.0001f;
				if (Vector3.SqrMagnitude(bounceVelocity) > BounceThreshold) {
					var distanceTraveled = Vector3.Magnitude(bone.currentTipPosition - bone.previousTipPosition);
					bone.previousTipPosition = bone.currentTipPosition - bounceVelocity;
					bone.currentTipPosition += Mathf.Max(0f, Vector3.Magnitude(bounceVelocity) - distanceTraveled) * Vector3.Normalize(bounceVelocity);
				} else {
					bone.previousTipPosition = bone.currentTipPosition;
				}
			}
			return hadCollision;
		}

		private static void ApplyAngleLimits(ref SpringBoneComponent bone, in SpringBoneProperties prop, float deltaTime) {
			if (prop.yAngleLimits.active == 0 && prop.zAngleLimits.active == 0)
				return;

			var origin = bone.position;
			var vector = bone.currentTipPosition - origin;

			var parentLocalToGlobal = bone.pivotLocalToGlobalMat;
			var forward = parentLocalToGlobal * -Vector3.right;

			var mulBack = parentLocalToGlobal * Vector3.back;
			var mulDown = parentLocalToGlobal * Vector3.down;

			if (prop.yAngleLimits.active > 0) {
				prop.yAngleLimits.ConstrainVector(
					vector,
					mulDown, //parentLocalToGlobal * -Vector3.up,
					mulBack, //parentLocalToGlobal * -Vector3.forward,
					forward, prop.angularStiffness, deltaTime);
			}

			if (prop.zAngleLimits.active > 0) {
				prop.zAngleLimits.ConstrainVector(
					vector,
					mulBack, //parentLocalToGlobal * -Vector3.forward,
					mulDown, //parentLocalToGlobal * -Vector3.up,
					forward, prop.angularStiffness, deltaTime);
			}

			bone.currentTipPosition = origin + vector;
		}

		private void UpdateRotation(ref SpringBoneComponent bone, in SpringBoneProperties prop, in Quaternion baseWorldRotation) {
			if (float.IsNaN(bone.currentTipPosition.x)
				| float.IsNaN(bone.currentTipPosition.y)
				| float.IsNaN(bone.currentTipPosition.z)) {
				bone.currentTipPosition = bone.position + baseWorldRotation * prop.boneAxis * prop.springLength;
				bone.previousTipPosition = bone.currentTipPosition;
			}

			var actualLocalRotation = ComputeLocalRotation(in baseWorldRotation, ref bone, in prop);
			bone.localRotation = Quaternion.Lerp(bone.localRotation, actualLocalRotation, this.settings.dynamicRatio);
			bone.rotation = bone.parentRotation * bone.localRotation;
		}

		private Quaternion ComputeLocalRotation(in Quaternion baseWorldRotation, ref SpringBoneComponent bone, in SpringBoneProperties prop) {
			var worldBoneVector = bone.currentTipPosition - bone.position;
			var localBoneVector = Quaternion.Inverse(baseWorldRotation) * worldBoneVector;
			localBoneVector = Vector3.Normalize(localBoneVector);

			var aimRotation = Quaternion.FromToRotation(prop.boneAxis, localBoneVector);
			var outputRotation = prop.initialLocalRotation * aimRotation;

			return outputRotation;
		}
	}
}
