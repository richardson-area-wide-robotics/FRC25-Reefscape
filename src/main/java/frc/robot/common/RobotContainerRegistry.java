package frc.robot.common;

import frc.robot.RobotContainer;
import frc.robot.common.annotations.Robot;
import frc.robot.common.interfaces.IRobotContainer;
import lombok.experimental.UtilityClass;

import java.io.File;
import java.net.URL;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@UtilityClass
public class RobotContainerRegistry {

    // A map to hold robot containers by team
    private static final Map<Integer, Class<?>> teamContainers = new HashMap<>();

    static {
        // Register the containers for each team
        for (Class<?> clazz : getAllClasses()) {
            if (clazz.isAnnotationPresent(Robot.class)) {
                Robot annotation = clazz.getAnnotation(Robot.class);
                teamContainers.put(annotation.team(), clazz);
            }
        }
    }

    public static IRobotContainer createContainerForTeam(int teamNumber) {
        // Try to fetch the container for the specific team
        Class<?> containerClass = teamContainers.get(teamNumber);

        // If not found, use the default container
        if (containerClass == null) {
            return RobotContainer.createContainer(); // Or return a fallback container if needed
        }

        try { //Create the container
            return (IRobotContainer) containerClass.getMethod("createContainer").invoke(null);
        } catch (Exception e) {
            e.printStackTrace();
        }

        return null; // or a default container if you prefer
    }

    private static final String PACKAGE_NAME = "frc.robot";  // Replace with your package

    private static List<Class<?>> getAllClasses() {
        List<Class<?>> classes = new ArrayList<>();
        String packagePath = PACKAGE_NAME.replace('.', '/');

        try {
            // Get the package URL
            URL resource = RobotContainerRegistry.class.getClassLoader().getResource(packagePath);
            if (resource == null) {
                throw new ClassNotFoundException("Package not found: " + PACKAGE_NAME);
            }

            // Convert URL to File and list files in the package
            File directory = new File(resource.toURI());
            if (!directory.exists()) {
                throw new ClassNotFoundException("Directory not found: " + directory.getPath());
            }

            // Loop through the files and load classes
            for (File file : directory.listFiles()) {
                if (file.getName().endsWith(".class")) {
                    // Get class name and load it using the ClassLoader
                    String className = PACKAGE_NAME + "." + file.getName().substring(0, file.getName().length() - 6);
                    Class<?> clazz = Class.forName(className);
                    classes.add(clazz);
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        return classes;
    }

}

