package frc.robot.subsystems;

import java.util.ArrayList;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PixyCamConstants;

public class PixyCamSubsystem extends SubsystemBase {
	private Pixy2 pixyCam;

	public PixyCamSubsystem() {
        pixyCam = Pixy2.createInstance(new SPILink());
        pixyCam.init();
        pixyCam.setLamp((byte)0, (byte)1);
        pixyCam.setLED(0, 30, 255);        
    }

	public Block getBiggestBlock(BallColor color) {
		// Gets the number of "blocks", identified targets, that match signature 1 on the Pixy2,
		// does not wait for new data if none is available,
		// and limits the number of returned blocks to 25, for a slight increase in efficiency
		int blockCount = pixyCam.getCCC().getBlocks(false, color.getSigmap(), 25);
		// Reports number of blocks found
		System.out.println("Found " + blockCount + " blocks!");
		
		if (blockCount <= 0) {
			// If blocks were not found, stop processing
			return null;
		}

		// Gets a list of all blocks found by the Pixy2
		ArrayList<Block> blocks = pixyCam.getCCC().getBlockCache();
		
		Block largestBlock = blocks.get(0);
		// Loops through all blocks and finds the widest one
		for (Block block : blocks) {
			if (block.getWidth() > largestBlock.getWidth()) {
				largestBlock = block;
			}
		}

		SmartDashboard.putNumber("Pixyblock.signature",  largestBlock.getSignature());
		SmartDashboard.putNumber("Pixyblock.X-value",  largestBlock.getX());
		SmartDashboard.putNumber("Pixyblock.Y-value",  largestBlock.getY());
		SmartDashboard.putNumber("Pixyblock.Witdth",  largestBlock.getWidth());
		SmartDashboard.putNumber("Pixyblock.Height",  largestBlock.getHeight());
		SmartDashboard.putNumber("Pixyblock.age",  largestBlock.getAge());

		return largestBlock;
	}

	/**
	 * Finds the angle to the nearest ball of the correct color.
	 * 
	 * @param color color of the ball pixy cam is seeking.
	 * @return null if no balls are found or the Rotation2d angle of the desired ball.
	 */
	public Rotation2d angleToBall(BallColor color){
		Block biggestBlock = getBiggestBlock(color);
		if (biggestBlock != null) {
			return Rotation2d.fromDegrees(
				(biggestBlock.getX() - PixyCamConstants.IMAGE_WIDTH_PIXELS / 2) * 
					(PixyCamConstants.FOV / PixyCamConstants.IMAGE_WIDTH_PIXELS)
			);
		} else {
			return null;
		}
	}

	public boolean hasBall(BallColor color) {
		return getBiggestBlock(color) != null;
	}

	public static enum BallColor {
		RED(Pixy2CCC.CCC_SIG2),
		BLUE(Pixy2CCC.CCC_SIG1);

		private final int sigmap;

		private BallColor(int sigmap) {
			this.sigmap = sigmap;
		}

		public int getSigmap() {
			return sigmap;
		}
        


	}
}

