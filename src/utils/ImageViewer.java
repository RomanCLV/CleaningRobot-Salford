package utils;

import sun.security.x509.EDIPartyName;

import java.awt.*;
import javax.swing.*;

public class ImageViewer
{
    private static JFrame editorFrame;
    private static ImageIcon imageIcon = new ImageIcon();

    public static void display(Image image)
    {
        System.out.println("ImageViewer display");

        display(image, false);
    }

    public static void display(Image image, boolean adjustSize)
    {
        System.out.println("ImageViewer display 2");

        imageIcon.setImage(image);

        if(adjustSize)
            editorFrame.setSize(new Dimension(image.getWidth(null), image.getHeight(null)));

        if(editorFrame != null)
            editorFrame.repaint();
    }

    public static void open(int width, int height, String title)
    {
        System.out.println("ImageViewer open");
        SwingUtilities.invokeLater(() ->
        {
            dispose();
            editorFrame = new JFrame(title);

            editorFrame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
            editorFrame.setDefaultCloseOperation(javax.swing.JFrame.EXIT_ON_CLOSE);

            editorFrame.setSize(width, height);
            editorFrame.setPreferredSize(new Dimension(width, height));

            JLabel jLabel = new JLabel();
            jLabel.setIcon(imageIcon);
            editorFrame.getContentPane().add(jLabel, BorderLayout.CENTER);

            editorFrame.pack();
            editorFrame.setLocationRelativeTo(null);
            editorFrame.setVisible(true);
        });
    }

    public static void dispose()
    {
        if (editorFrame != null)
        {
            editorFrame.dispose();
            System.out.println("ImageViewer dispose");
        }
    }
}
