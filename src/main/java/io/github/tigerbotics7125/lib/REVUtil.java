package io.github.tigerbotics7125.lib;

import com.revrobotics.REVLibError;
import java.util.function.Supplier;

public class REVUtil {

    public static void retryFailable(int retries, Supplier<REVLibError> failable) {
        System.out.println("Trying failable REV action...");
        int i = 0;
        do {
            i++;
            if (i > retries) {
                System.out.println(String.format("FAILABLE ACTION STILL FAILED AFTER %d TRIES", retries));
                return;
            }
        } while (failable.get() != REVLibError.kOk);
        System.out.println("Failable action succeded!");
    }

}
