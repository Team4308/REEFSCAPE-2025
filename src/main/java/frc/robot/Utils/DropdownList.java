package frc.robot.Utils;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

/**
 * The {@link SendableChooser} class is a useful tool for presenting a selection
 * of options to the
 * {@link SmartDashboard}.
 *
 * <p>
 * For instance, you may wish to be able to select between multiple autonomous
 * modes. You can do
 * this by putting every possible Command you want to run as an autonomous into
 * a {@link
 * SendableChooser} and then put it into the {@link SmartDashboard} to have a
 * list of options appear
 * on the laptop. Once autonomous starts, simply ask the {@link SendableChooser}
 * what the selected
 * value is.
 *
 * @param <V> The type of the values to be stored
 */
public class DropdownList<V> implements Sendable, AutoCloseable {
    /** The key for the default value. */
    private static final String DEFAULT = "default";

    /** The key for the selected option. */
    private static final String SELECTED = "selected";

    /** The key for the active option. */
    private static final String ACTIVE = "active";

    /** The key for the option array. */
    private static final String OPTIONS = "options";

    /** The key for the instance number. */
    private static final String INSTANCE = ".instance";

    /** A map linking strings to the objects they represent. */
    private final ArrayList<V> m_map = new ArrayList<>();

    private String m_defaultChoice = "None";
    private final int m_instance;
    private String m_previousVal;
    private Consumer<String> m_listener;
    private static final AtomicInteger s_instances = new AtomicInteger();

    /** Instantiates a {@link SendableChooser}. */
    @SuppressWarnings("this-escape")
    public DropdownList() {
        m_instance = s_instances.getAndIncrement();
        SendableRegistry.add(this, "SendableChooser", m_instance);
    }

    @Override
    public void close() {
        SendableRegistry.remove(this);
    }

    /**
     * Adds the given object to the list of options. On the {@link SmartDashboard}
     * on the desktop, the
     * object will appear as the given name.
     *
     * @param name   the name of the option
     * @param object the option
     */
    public void addOption(V object) {
        m_map.add(object);
    }

    /**
     * Adds the given object to the list of options and marks it as the default.
     * Functionally, this is
     * very close to {@link #addOption(String, Object)} except that it will use this
     * as the default
     * option if none other is explicitly selected.
     *
     * @param name   the name of the option
     * @param object the option
     */
    public void setDefaultOption(V object) {
        addOption(object);
    }

    /**
     * Returns the selected option. If there is none selected, it will return the
     * default. If there is
     * none selected and no default, then it will return {@code null}.
     *
     * @return the option selected
     */
    public String getSelected() {
        m_mutex.lock();
        try {
            if (m_selected != null) {
                return m_selected;
            } else {
                return m_defaultChoice;
            }
        } finally {
            m_mutex.unlock();
        }
    }

    /**
     * Bind a listener that's called when the selected value changes. Only one
     * listener can be bound.
     * Calling this function will replace the previous listener.
     *
     * @param listener The function to call that accepts the new value
     */
    public void onChange(Consumer<String> listener) {
        requireNonNullParam(listener, "listener", "onChange");
        m_mutex.lock();
        m_listener = listener;
        m_mutex.unlock();
    }

    public void setSelected(String value) {
        m_selected = value;
    }

    private String m_selected;
    private final ReentrantLock m_mutex = new ReentrantLock();

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("String Chooser");
        builder.publishConstInteger(INSTANCE, m_instance);
        builder.addStringProperty(DEFAULT, () -> m_defaultChoice, null);
        builder.addStringArrayProperty(OPTIONS, () -> m_map.toArray(new String[0]), null);
        builder.addStringProperty(
                ACTIVE,
                () -> {
                    m_mutex.lock();
                    try {
                        if (m_selected != null) {
                            return m_selected;
                        } else {
                            return m_defaultChoice;
                        }
                    } finally {
                        m_mutex.unlock();
                    }
                },
                null);
        builder.addStringProperty(
                SELECTED,
                null,
                val -> {
                    Consumer<String> listener;
                    m_mutex.lock();
                    try {
                        m_selected = val;
                        if (!m_selected.equals(m_previousVal) && m_listener != null) {
                            listener = m_listener;
                        } else {
                            listener = null;
                        }
                        m_previousVal = val;
                    } finally {
                        m_mutex.unlock();
                    }
                    if (listener != null) {
                        listener.accept(m_selected);
                    }
                });
    }
}
