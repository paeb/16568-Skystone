package teamcode2.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

import static android.graphics.Bitmap.createBitmap;
import static android.graphics.Bitmap.createScaledBitmap;

//import org.firstinspires.ftc.teamcode.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
public class SkystoneDetection {

    VuforiaLocalizer vuforia;
    private int step = 1;

    private Image rgbImage = null;
    private int rgbTries = 0;

    private double yellowCountL = 1;
    private double yellowCountC = 1;
    private double yellowCountR = 1;

    private double blackCountL = 1;
    private double blackCountC = 1;
    private double blackCountR = 1;
    VuforiaLocalizer.CloseableFrame closeableFrame = null;
    private skystonePos pos;
    public static double blackYellowRatioL = 0;
    public static double blackYellowRatioC = 0;
    public static double blackYellowRatioR = 0;
    private Bitmap bitmap;

    public SkystoneDetection(VuforiaLocalizer vuforia) {
        this.vuforia = vuforia;
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        this.vuforia.setFrameQueueCapacity(1);
    }

    public enum skystonePos {
        LEFT, CENTER, RIGHT
    }

    public String vuforiascan(boolean saveBitmaps, boolean red) {
        switch (step) {
            case 1:
                if (rgbImage == null) {
                    try {
                        closeableFrame = this.vuforia.getFrameQueue().take();
                        long numImages = closeableFrame.getNumImages();

                        for (int i = 0; i < numImages; i++) {
                            if (closeableFrame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                                rgbImage = closeableFrame.getImage(i);
                                if (rgbImage != null) {
                                    break;
                                }
                            }
                        }
                    } catch (InterruptedException exc) {
                    } finally {
                        if (closeableFrame != null) closeableFrame.close();
                    }
                }
                else { //once image is captured
                    step = 2;
                }
                break;
            case 2:
                // copy the bitmap from the Vuforia frame
                try {
                    bitmap = Bitmap.createBitmap(rgbImage.getWidth(), rgbImage.getHeight(), Bitmap.Config.RGB_565);
                    bitmap.copyPixelsFromBuffer(rgbImage.getPixels());
                }
                catch (Exception e) {
                    step = 1; //go back to step 1
                    break;
                }

                String path = Environment.getExternalStorageDirectory().toString();
                FileOutputStream out = null;

                String bitmapName;
                String croppedBitmapName;

                if (red) {
                    bitmapName = "BitmapRED.png";
                    croppedBitmapName = "BitmapCroppedRED.png";
                } else {
                    bitmapName = "BitmapBLUE.png";
                    croppedBitmapName = "BitmapCroppedBLUE.png";
                }

                //Save bitmap to file
                if (saveBitmaps) {
                    try {
                        File file = new File(path, bitmapName);
                        out = new FileOutputStream(file);
                        bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
                    } catch (Exception e) {
                        e.printStackTrace();
                    } finally {
                        try {
                            if (out != null) {
                                out.flush();
                                out.close();
                            }
                        } catch (IOException e) {
                            e.printStackTrace();
                        }
                    }
                }

                int cropStartX;
                int cropStartY;
                int cropWidth;
                int cropHeight;

                if (red) {
                    cropStartX = (int) ((150.0 / 720.0) * bitmap.getWidth());
                    cropStartY = (int) ((100.0 / 480.0) * bitmap.getHeight());
                    cropWidth = (int) ((530.0 / 720.0) * bitmap.getWidth());
                    cropHeight = (int) ((150.0 / 480.0) * bitmap.getHeight());
                } else {
                    cropStartX = (int) ((370.0 / 1280.0) * bitmap.getWidth());
                    cropStartY = (int) ((170.0 / 720.0) * bitmap.getHeight());
                    cropWidth = (int) ((890.0 / 1280.0) * bitmap.getWidth());
                    cropHeight = (int) ((125.0 / 720.0) * bitmap.getHeight());
                }

                //log error messages
                /**
                 DbgLog.msg("10435 vuforiascan"
                 + " cropStartX: " + cropStartX
                 + " cropStartY: " + cropStartY
                 + " cropWidth: " + cropWidth
                 + " cropHeight: " + cropHeight
                 + " Width: " + bitmap.getWidth()
                 + " Height: " + bitmap.getHeight()
                 );
                 */

                bitmap = createBitmap(bitmap, cropStartX, cropStartY, cropWidth, cropHeight); //Cropped Bitmap to show only stones

                // Save cropped bitmap to file
                if (saveBitmaps) {
                    try {
                        File file = new File(path, croppedBitmapName);
                        out = new FileOutputStream(file);
                        bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
                    } catch (Exception e) {
                        e.printStackTrace();
                    } finally {
                        try {
                            if (out != null) {
                                out.flush();
                                out.close();
                            }
                        } catch (IOException e) {
                            e.printStackTrace();
                        }
                    }
                }
                bitmap = createScaledBitmap(bitmap, 110, 20, true); //Compress bitmap to reduce scan time

                int height;
                int width;
                int pixel;
                int bitmapWidth = bitmap.getWidth();
                int bitmapHeight = bitmap.getHeight();
                int colWidth = (int) ((double) bitmapWidth / 6.0);
                int colorLStartCol = (int) ((double) bitmapWidth * (1.0 / 6.0) - ((double) colWidth / 2.0));
                int colorCStartCol = (int) ((double) bitmapWidth * (3.0 / 6.0) - ((double) colWidth / 2.0));
                int colorRStartCol = (int) ((double) bitmapWidth * (5.0 / 6.0) - ((double) colWidth / 2.0));

                for (height = 0; height < bitmapHeight; ++height) {
                    for (width = colorLStartCol; width < colorLStartCol + colWidth; ++width) {
                        pixel = bitmap.getPixel(width, height);
                        if (Color.red(pixel) < 200 || Color.green(pixel) < 200 || Color.blue(pixel) < 200) {
                            yellowCountL += Color.red(pixel);
                            blackCountL += Color.blue(pixel);
                        }

                    /*
                    if (Color.red(pixel) > 120 && Color.green(pixel) > 80 && Color.blue(pixel) < 20) {
                        yellowCountL += 1;
                    } else if (Color.red(pixel) < 120 && Color.green(pixel) < 120 && Color.blue(pixel) < 120) {
                        blackCountL += 1;
                    }
                     */

                        //colorcountL += Color.red(pixel) + Color.green(pixel) + Color.blue(pixel);
                    }
                    for (width = colorCStartCol; width < colorCStartCol + colWidth; ++width) {
                        pixel = bitmap.getPixel(width, height);

                        if (Color.red(pixel) < 200 || Color.green(pixel) < 200 || Color.blue(pixel) < 200) {
                            yellowCountC += Color.red(pixel);
                            blackCountC += Color.blue(pixel);
                        }
                    /*
                    if (Color.red(pixel) > 120 && Color.green(pixel) > 80 && Color.blue(pixel) < 20) {
                        yellowCountC += 1;
                    } else if (Color.red(pixel) < 120 && Color.green(pixel) < 120 && Color.blue(pixel) < 120) {
                        blackCountC += 1;
                    }
                    */
                        //colorcountC += Color.red(pixel) + Color.green(pixel) + Color.blue(pixel);
                    }

                    for (width = colorRStartCol; width < colorRStartCol + colWidth; ++width) {
                        pixel = bitmap.getPixel(width, height);

                        if (Color.red(pixel) < 200 || Color.green(pixel) < 200 || Color.blue(pixel) < 200) {
                            yellowCountR += Color.red(pixel);
                            blackCountR += Color.blue(pixel);
                        }
                    /*
                    if (Color.red(pixel) > 120 && Color.green(pixel) > 80 && Color.blue(pixel) < 20) {
                        yellowCountR += 1;
                    } else if (Color.red(pixel) < 120 && Color.green(pixel) < 120 && Color.blue(pixel) < 120) {
                        blackCountR += 1;
                    }
                    */
                        //colorcountR += Color.red(pixel) + Color.green(pixel) + Color.blue(pixel);
                    }
                }
                blackYellowRatioL = blackCountL / yellowCountL;
                blackYellowRatioC = blackCountC / yellowCountC;
                blackYellowRatioR = blackCountR / yellowCountR;

                if (blackYellowRatioL > blackYellowRatioC && blackYellowRatioL > blackYellowRatioR) {
                    pos = skystonePos.RIGHT;
                }
                else if (blackYellowRatioC >= blackYellowRatioL && blackYellowRatioC >= blackYellowRatioR) {
                    pos = skystonePos.CENTER;
                }
                else { //if r is the greatest
                    pos = skystonePos.LEFT;
                }

                //telemetry and print out the raitos fort righty

                /**
                if (blackYellowRatioC > blackYellowRatioL && blackYellowRatioC > blackYellowRatioR) {
                    pos = skystonePos.CENTER;
                }
                if (blackYellowRatioR > blackYellowRatioL && blackYellowRatioR > blackYellowRatioC) {
                    pos = skystonePos.RIGHT;
                }
                 */
                /**
                 * if (blackYellowRatioR > blackYellowRatioL && blackYellowRatioR > blackYellowRatioC) {
                 *                         pos = skystonePos.RIGHT;
                 *                     }
                 *                     else {
                 *                         pos = skystonePos.LEFT;
                 *                     }
                 */
                step = 3;
                break;
        }
        if (step == 3) {
            return pos.toString();
        }
        else {
            return "NONE";
        }
    }
}