package frc.robot.util;

public class SubsystemDataProcessor implements Runnable {
    public interface IoRefresher {
        void refreshData();
    }

    public interface DataReader {
        void readData();
    }

    public static void createSubsystemDataProcessor(IoRefresher ioRefresher, DataReader dataReader) {
        Thread t = new Thread(new SubsystemDataProcessor(ioRefresher, dataReader));
        t.setDaemon(true);
        t.setName("SubsystemDataProcessor");
        t.start();
    }

    private final double looptime = 20.0;
    private final IoRefresher ioRefresher;
    private final DataReader dataReader;

    public SubsystemDataProcessor(IoRefresher ioRefresher, DataReader dataReader) {
        this.ioRefresher = ioRefresher;
        this.dataReader = dataReader;
    }

    @Override
    public void run() {
        while (true) {
            double timeStamp = System.currentTimeMillis();
            ioRefresher.refreshData();
            dataReader.readData();

            try {
                long diff = System.currentTimeMillis() - (long) timeStamp;
                if (diff < looptime) {
                    Thread.sleep((long) (looptime - diff));
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }
}
