package io.github.tigerbotics7125.lib;

import com.revrobotics.REVLibError;
import java.util.function.Supplier;

public class REVUtil {

    public static void retryFailable(int retries, Supplier<REVLibError> failable) {
        System.out.println("Trying failable action...");
        int i = 0;
        REVLibError err = failable.get();
        while (err != REVLibError.kOk) {
            i++;

            // remove leading k.
            String errName = err.name().substring(1);

            System.out.printf("Failable action failed (attempt: %d reason: %s)\n", i, errName);

            if (i >= retries) {
                System.out.println("DID NOT COMPLETE FAILABLE ACTION");
                return;
            }
        }
        System.out.println("Failable action succeded!");
    }

}
