package frc.robot.util;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.UUID;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/** Gravação física: USB primeiro, reserva interna limitada. IO só na thread do Logger. */
public final class RobotLogWriter implements LogDataReceiver {
    // Precisa ficar ABAIXO de 50 MB, onde a REVLib começa a apagar os logs dela.
    // Com 100 MiB, o disco parou entre 84 e 99 MB livres e o robô passou 5 sessões
    // seguidas sem log nenhum (29 e 30/10/2026).
    static final long RESERVE_BYTES = 30L * 1024 * 1024;
    static final long INTERNAL_FILE_LIMIT_BYTES = 200L * 1024 * 1024;
    // O DataLogWriter da WPILib só grava blocos completos de 16 KiB: depois
    // das primeiras tabelas o arquivo ainda tem 0 bytes, mesmo com flush
    // (medido em 29/10/2026). Conferir o tamanho na 1ª tabela parava a
    // gravação. O tamanho só é conferido depois de 5 s gravando.
    static final long CHECK_PERIOD_NANOS = 5_000_000_000L;
    private static final DateTimeFormatter NAME_TIME = DateTimeFormatter.ofPattern("yy-MM-dd_HH-mm-ss");
    public record Status(String state, String path, double freeMiB, boolean writing) {
        /** Desistiu de gravar. Iniciando e conferindo o arquivo ainda não são falha. */
        public boolean failed() {
            return !writing && !state.equals("NAO_INICIADO") && !state.equals("VERIFICANDO_ARQUIVO");
        }
    }
    private static volatile Status status = new Status("NAO_INICIADO", "", -1, false);
    private final Path usbRoot;
    private final Path internalRoot;
    private WPILOGWriter writer;
    private Path file;
    private boolean internal;
    private boolean active;
    private boolean fileConfirmed;
    private boolean named;
    private long nextCheckNanos;

    public RobotLogWriter() {
        this(Path.of("/U"), Path.of("/home/lvuser/logs/advantagekit"));
    }

    RobotLogWriter(Path usbRoot, Path internalRoot) {
        this.usbRoot = usbRoot;
        this.internalRoot = internalRoot;
    }

    public static Status status() { return status; }

    @Override public void start() {
        // Não cria /U: sem USB montado, esse diretório pode nem existir.
        Path directory = Files.isDirectory(usbRoot) ? prepare(usbRoot.resolve("logs")) : null;
        internal = directory == null;
        if (internal) directory = prepare(internalRoot);
        if (directory == null) {
            status = new Status("SEM_ESPACO_OU_SEM_PERMISSAO", "", freeMiB(internalRoot), false);
            return;
        }
        file = directory.resolve("FRC9460_" + UUID.randomUUID() + ".wpilog");
        writer = new WPILOGWriter(file.toString(), WPILOGWriter.AdvantageScopeOpenBehavior.NEVER);
        writer.start();
        active = true;
        fileConfirmed = false;
        named = false;
        nextCheckNanos = System.nanoTime() + CHECK_PERIOD_NANOS;
        status = new Status("VERIFICANDO_ARQUIVO", file.toString(), -1, false);
    }

    private static double freeMiB(Path directory) {
        try {
            return Files.getFileStore(directory).getUsableSpace() / 1048576.0;
        } catch (IOException | SecurityException ex) {
            return -1;
        }
    }

    private static Path prepare(Path directory) {
        try {
            Files.createDirectories(directory);
            if (Files.getFileStore(directory).getUsableSpace() < RESERVE_BYTES) return null;
            Path probe = Files.createTempFile(directory, ".write-test-", ".tmp");
            Files.delete(probe);
            return directory;
        } catch (IOException | SecurityException ex) {
            return null;
        }
    }

    @Override public void putTable(LogTable table) {
        if (!active) return;
        try {
            writer.putTable(table); // Grava em blocos de 16 KiB (CHECK_PERIOD_NANOS).
            if (!fileConfirmed) {
                // 1ª tabela: o WPILOGWriter criou o arquivo? Se não abriu, não há o que esperar.
                if (!Files.isRegularFile(file)) {
                    stop("FALHA_DE_GRAVACAO", -1);
                    return;
                }
                fileConfirmed = true;
                setWriting(Files.getFileStore(file).getUsableSpace());
            }
            long now = System.nanoTime();
            if (now >= nextCheckNanos) {
                nextCheckNanos = now + CHECK_PERIOD_NANOS;
                nameWithDate(table);
                long free = Files.getFileStore(file).getUsableSpace();
                long size = Files.size(file);
                // 5 s de tabelas do robô passam de 16 KiB: vazio aqui é falha real.
                if (size < 6) stop("FALHA_DE_GRAVACAO", free);
                else if (free < RESERVE_BYTES) stop("RESERVA_DE_DISCO_ATINGIDA", free);
                else if (internal && size >= INTERNAL_FILE_LIMIT_BYTES) stop("LIMITE_INTERNO_200_MIB", free);
                else setWriting(free);
            }
        } catch (IOException | RuntimeException ex) {
            stop("FALHA_DE_GRAVACAO", -1);
        }
    }

    static String datedName(LocalDateTime time) {
        return "FRC9460_" + NAME_TIME.format(time) + ".wpilog";
    }

    /**
     * Com o relógio acertado pela Driver Station, troca o nome pela data (UTC do
     * roboRIO): com UUID não dava para saber de qual sessão era cada log ao baixar.
     */
    private void nameWithDate(LogTable table) {
        if (named || !table.get("DriverStation/DSAttached", false)
                || !table.get("SystemStats/SystemTimeValid", false)) return;
        named = true;
        try {
            Path dated = file.resolveSibling(datedName(LocalDateTime.now()));
            Files.move(file, dated);
            file = dated;
        } catch (IOException | RuntimeException ex) {
            // Segue gravando com o nome atual: o log vale mais que o nome.
        }
    }

    private void setWriting(long free) {
        status = new Status(internal ? "GRAVANDO_INTERNO" : "GRAVANDO_USB",
            file.toString(), free / 1048576.0, true);
    }

    private void stop(String state, long free) {
        active = false;
        status = new Status(state, file == null ? "" : file.toString(),
            free < 0 ? -1 : free / 1048576.0, false);
        // O WPILOGWriter 26.0.2 pode estar sem DataLogWriter após falha em start().
        try { if (writer != null) writer.end(); } catch (RuntimeException ignored) { }
    }

    @Override public void end() {
        if (active) stop("ENCERRADO", -1);
    }
}
