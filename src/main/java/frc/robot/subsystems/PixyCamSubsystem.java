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

// public Block getBlueBiggestBlock() {
// 	// Gets the number of "blocks", identified targets, that match signature 1 on the Pixy2,
// 	// does not wait for new data if none is available,
// 	// and limits the number of returned blocks to 25, for a slight increase in efficiency
// 	int blockCount = m_pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);
// 	System.out.println("Found " + blockCount + " blocks!"); // Reports number of blocks found
// 	if (blockCount <= 0) {
// 		return null; // If blocks were not found, stop processing
// 	}
// 	ArrayList<Block> blocks = m_pixy.getCCC().getBlockCache(); // Gets a list of all blocks found by the Pixy2
// 	Block largestBlock = null;
// 	for (Block block : blocks) { // Loops through all blocks and finds the widest one
// 		SmartDashboard.putNumber("Pixyblock.signature",  block.getSignature());
// 		SmartDashboard.putNumber("Pixyblock.X-value",  block.getX());
// 		SmartDashboard.putNumber("Pixyblock.Y-value",  block.getY());
// 		SmartDashboard.putNumber("Pixyblock.Witdth",  block.getWidth());
// 		SmartDashboard.putNumber("Pixyblock.Height",  block.getHeight());
// 		SmartDashboard.putNumber("Pixyblock.age",  block.getAge());

// 		if (largestBlock == null) {
// 			largestBlock = block;
// 		} else if (block.getWidth() > largestBlock.getWidth()) {
// 			largestBlock = block;
// 		}
// 	}
// 	return largestBlock;
// }

// public Block getRedBiggestBlock() {
// 	// Gets the number of "blocks", identified targets, that match signature 1 on the Pixy2,
// 	// does not wait for new data if none is available,
// 	// and limits the number of returned blocks to 25, for a slight increase in efficiency
// 	int blockCount = m_pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG2, 25);
// 	System.out.println("Found " + blockCount + " blocks!"); // Reports number of blocks found
// 	if (blockCount <= 0) {
// 		return null; // If blocks were not found, stop processing
// 	}
// 	ArrayList<Block> blocks = m_pixy.getCCC().getBlockCache(); // Gets a list of all blocks found by the Pixy2
// 	Block largestBlock = null;
// 	for (Block block : blocks) { // Loops through all blocks and finds the widest one
// 		if (largestBlock == null) {
// 			largestBlock = block;
// 		} else if (block.getWidth() > largestBlock.getWidth()) {
// 			largestBlock = block;
// 		}
// 	}
// 	return largestBlock;
// }

// public double angleToBlueBall(){
// 	Block biggestBlock = getBlueBiggestBlock();
// 	if (biggestBlock == null) {
// 		return 0;
// 	} else {
// 		//return Math.abs(((biggestBlock.getX() - 158) / (316 / 60)));
// 		return biggestBlock.getX()-150*60;
// 	}
// }

// public boolean hasBlueBall() {
// 	return getBlueBiggestBlock() != null;
// }

// public double angleToRedBall(){
// 	Block biggestBlock = getRedBiggestBlock();
// 	if (biggestBlock == null) {
// 		return 0;
// 	} else {
// 		//return Math.abs(((- 158)/(316/60)));
// 		return (biggestBlock.getX() - PixyCamConstants.IMAGE_WIDTH_PIXELS / 2) * 
// 			(PixyCamConstants.FOV / PixyCamConstants.IMAGE_WIDTH_PIXELS);
// 	}
// }

// public boolean hasRedBall() {
// 	return getRedBiggestBlock() != null;
// }