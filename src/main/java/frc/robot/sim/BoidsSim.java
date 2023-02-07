package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

/** Simulate Boids effect in Field2d */
public class BoidsSim {

	private static final double WIDTH = 15.2;
	private static final double HEIGHT = 8.2;

	private final Field2d field2d;

	private List<Boid> boids;

	// Configuration values
	private final int boidsAmount;
	private final double visualRange; // How far boids can see other boids
	private final double margin; // (keepWithinBounds) Distance from edge of wall
	private final double turnFactor; // (keepWithinBounds) Speed at which boids rebound from wall
	private final double centeringFactor; // (flyTowardsCenter) adjust velocity by this %
	private final double minDistance; // (avoidOthers) The distance to stay away from other boids
	private final double avoidFactor; // (avoidOthers) Adjust velocity by this %
	private final double matchingFactor; // (matchingVelocity) Adjust by this % of average velocity
	private final double speedLimit; // (limitSpeed) Max speed of boids

	public BoidsSim(
			Field2d field2d,
			int boidsAmount,
			double visualRange,
			double margin,
			double turnFactor,
			double centeringFactor,
			double minDistance,
			double avoidFactor,
			double matchingFactor,
			double speedLimit) {
		this.field2d = field2d;
		this.boidsAmount = boidsAmount;
		this.visualRange = visualRange;
		this.margin = margin;
		this.turnFactor = turnFactor;
		this.centeringFactor = centeringFactor;
		this.minDistance = minDistance;
		this.avoidFactor = avoidFactor;
		this.matchingFactor = matchingFactor;
		this.speedLimit = speedLimit;

		boids = new ArrayList<>();
		for (int i = 0; i < boidsAmount; i++) {
			boids.add(
					new Boid(
							Math.random() * WIDTH,
							Math.random() * HEIGHT,
							Math.random() * 1 - 5,
							Math.random() * 1 - 5,
							0.0));
		}
	}

	private double distance(Boid boid1, Boid boid2) {
		return Math.sqrt(
				(boid1.x - boid2.x) * (boid1.x - boid2.x)
						+ (boid1.y - boid2.y) * (boid1.y - boid2.y));
	}

	private List<Boid> nClosestBoids(Boid boid, int n) {
		List<Boid> copy = boids.subList(0, boids.size());
		copy.sort((a, b) -> (int) (distance(boid, a) - distance(boid, b)));
		return copy.subList(1, n + 1);
	}

	private void keepWithinBounds(Boid boid) {
		if (boid.x < margin) {
			boid.dx += turnFactor;
		}
		if (boid.x > WIDTH - margin) {
			boid.dx -= turnFactor;
		}
		if (boid.y < margin) {
			boid.dy += turnFactor;
		}
		if (boid.y > HEIGHT - margin) {
			boid.dy -= turnFactor;
		}
	}

	private void flyTowardsCenter(Boid boid) {
		double centerX = 0;
		double centerY = 0;
		int numNeighbors = 0;

		for (Boid otherBoid : boids) {
			if (distance(boid, otherBoid) < visualRange) {
				centerX += otherBoid.x;
				centerY += otherBoid.y;
				numNeighbors += 1;
			}
		}

		if (numNeighbors != 0) {
			centerX = centerX / numNeighbors;
			centerY = centerY / numNeighbors;

			boid.dx += (centerX - boid.x) * centeringFactor;
			boid.dy += (centerY - boid.y) * centeringFactor;
		}
	}

	// Move away from other boids that are too close to avoid colliding
	private void avoidOthers(Boid boid) {
		double moveX = 0;
		double moveY = 0;

		for (Boid otherBoid : boids) {
			if (otherBoid != boid) {
				if (distance(boid, otherBoid) < minDistance) {
					moveX += boid.x - otherBoid.x;
					moveY += boid.y - otherBoid.y;
				}
			}
		}

		boid.dx += moveX * avoidFactor;
		boid.dy += moveY * avoidFactor;
	}

	// Find the average velocity (speed and direction) of the other boids and
	// adjust velocity slightly to match.
	private void matchVelocity(Boid boid) {
		double avgDX = 0;
		double avgDY = 0;
		double numNeighbors = 0;

		for (Boid otherBoid : boids) {
			if (distance(boid, otherBoid) < visualRange) {
				avgDX += otherBoid.dx;
				avgDY += otherBoid.dy;
				numNeighbors += 1;
			}
		}

		if (numNeighbors != 0) {
			avgDX = avgDX / numNeighbors;
			avgDY = avgDY / numNeighbors;

			boid.dx += (avgDX - boid.dx) * matchingFactor;
			boid.dy += (avgDY - boid.dy) * matchingFactor;
		}
	}

	// Speed will naturally vary in flocking behavior, but real animals can't go
	// arbitrarily fast.
	private void limitSpeed(Boid boid) {
		double speed = Math.sqrt(boid.dx * boid.dx + boid.dy * boid.dy);
		if (speed > speedLimit) {
			boid.dx = (boid.dx / speed) * speedLimit;
			boid.dy = (boid.dy / speed) * speedLimit;
		}
	}

	private void adjustHeading(Boid boid) {
		boid.rot = Math.atan2(boid.dy, boid.dx);
	}

	public void update() {
		for (Boid boid : boids) {
			flyTowardsCenter(boid);
			avoidOthers(boid);
			matchVelocity(boid);
			limitSpeed(boid);
			adjustHeading(boid);
			keepWithinBounds(boid);

			// Update the position based on the current velocity
			boid.x += boid.dx;
			boid.y += boid.dy;
		}

		field2d.getObject("Boids")
				.setPoses(
						boids.stream()
								.map(
										boid ->
												new Pose2d(
														boid.x,
														boid.y,
														Rotation2d.fromRadians(boid.rot)))
								.collect(Collectors.toList()));
	}

	static class Boid {
		double x;
		double y;
		double dx;
		double dy;
		double rot;

		public Boid(double x, double y, double dx, double dy, double rot) {
			this.x = x;
			this.y = y;
			this.dx = dx;
			this.dy = dy;
			this.rot = rot;
		}
	}
}
