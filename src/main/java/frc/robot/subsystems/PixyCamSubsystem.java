package frc.robot.subsystems;

import java.util.ArrayList;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PixyCamSubSystem extends SubsystemBase {

    private Pixy2 m_pixy;

    public PixyCamSubSystem(){
        m_pixy = Pixy2.createInstance(new SPILink());
        m_pixy.init();
        m_pixy.setLamp((byte)0,(byte)1);
        m_pixy.setLED(0,30,255);        
    }

    public Block getBlueBiggestBlock() {
		// Gets the number of "blocks", identified targets, that match signature 1 on the Pixy2,
		// does not wait for new data if none is available,
		// and limits the number of returned blocks to 25, for a slight increase in efficiency
		int blockCount = m_pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);
		System.out.println("Found " + blockCount + " blocks!"); // Reports number of blocks found
		if (blockCount <= 0) {
			return null; // If blocks were not found, stop processing
		}
		ArrayList<Block> blocks = m_pixy.getCCC().getBlockCache(); // Gets a list of all blocks found by the Pixy2
		Block largestBlock = null;
		for (Block block : blocks) { // Loops through all blocks and finds the widest one
			SmartDashboard.putNumber("Pixyblock.signature",  block.getSignature());
			SmartDashboard.putNumber("Pixyblock.Angle", block.getX());
			SmartDashboard.putNumber("Pixyblock.X-value",  block.getX());
			SmartDashboard.putNumber("Pixyblock.Y-value",  block.getY());
			SmartDashboard.putNumber("Pixyblock.Witdth",  block.getWidth());
			SmartDashboard.putNumber("Pixyblock.Height",  block.getHeight());
			SmartDashboard.putNumber("Pixyblock.age",  block.getAge());

			if (largestBlock == null) {
				largestBlock = block;
			} else if (block.getWidth() > largestBlock.getWidth()) {
				largestBlock = block;
			}
		}
		return largestBlock;
	}
	public Block getRedBiggestBlock() {
		// Gets the number of "blocks", identified targets, that match signature 1 on the Pixy2,
		// does not wait for new data if none is available,
		// and limits the number of returned blocks to 25, for a slight increase in efficiency
		int blockCount = m_pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG2, 25);
		System.out.println("Found " + blockCount + " blocks!"); // Reports number of blocks found
		if (blockCount <= 0) {
			return null; // If blocks were not found, stop processing
		}
		ArrayList<Block> blocks = m_pixy.getCCC().getBlockCache(); // Gets a list of all blocks found by the Pixy2
		Block largestBlock = null;
		for (Block block : blocks) { // Loops through all blocks and finds the widest one
			if (largestBlock == null) {
				largestBlock = block;
			} else if (block.getWidth() > largestBlock.getWidth()) {
				largestBlock = block;
			}
		}
		return largestBlock;
	}
	
	public double angleToBlueBall(){
		Block biggestBlock = getBlueBiggestBlock();
		if (biggestBlock == null) {
			return 0;
		} else {
			//return Math.abs(((biggestBlock.getX() - 158) / (316 / 60)));
			return biggestBlock.getX()-150*60;
		}
	}

	public boolean hasBlueBall() {
		return getBlueBiggestBlock() != null;
	}

	public double angleToRedBall(){
		Block biggestBlock = getRedBiggestBlock();
		if (biggestBlock == null) {
			return 0;
		}else{
			return Math.abs(((- 158)/(316/60)));
		}
	}

	public boolean hasRedBall() {
		return getRedBiggestBlock() != null;
	}
}