package frc.robot.util.swerve;

import java.lang.reflect.Array;
import java.lang.reflect.Constructor;

import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.SortedMap;
import java.util.TreeMap;

import frc.robot.subsystems.SwerveSubsystem;

public abstract class SocketController<
    SocketNamerator extends Enum<SocketNamerator>,
    SocketType extends Socket
> {
    private final Constructor<SocketType> socketConstructor;
    private final Class<SocketType> socketType;

    private final SwerveSubsystem swerve;
    private final SortedMap<
        SocketNamerator,
        SocketType
    > socketInstances;
    private final HashSet<SocketType> acknowledgements;

    public SocketController(
        SwerveSubsystem swerve,
        Class<SocketNamerator> socketNames,
        Class<SocketType> socketType
    ) {
        this.swerve = swerve;

        this.socketType = socketType;
        try {
            this.socketConstructor = socketType.getDeclaredConstructor(
                this.getClass(),
                swerve.getClass()
            );
        } catch (Exception e ) {
            throw new RuntimeException(
                "failed to get genericified socket constructor: \n"
                + e
            );
        }

        this.socketInstances = new TreeMap<>(
            new Comparator<SocketNamerator>() {
                @Override
                public int compare(
                        SocketNamerator one,
                        SocketNamerator other
                ) {
                    SocketType socketOne = socketInstances.get(one);
                    SocketType socketOther = socketInstances.get(other);

                    if (socketOne.isPossessed() == socketOther.isPossessed()) {
                        if (
                            socketOne.isRequestingActive()
                            == socketOther.isRequestingActive()
                        ) {
                            return (one.ordinal() < other.ordinal())
                                ? 1
                                : -1;
                        } else {
                            return socketOne.isRequestingActive()
                                ? 1
                                : -1;
                        }
                    } else {
                        return socketOne.isPossessed()
                            ? 1
                            : -1;
                    }
                }
            }
        );
        this.acknowledgements = new HashSet<>();
        for (SocketNamerator name : socketNames.getEnumConstants()) {
            SocketType newSocket = newSocketInstance();
            socketInstances.put(
                name,
                newSocket
            );
            acknowledgements.add(newSocket);
        }
    }

    public abstract SocketType getActiveSocket();

    private SocketType newSocketInstance() {
        try {
            return socketConstructor.newInstance(
                this,
                swerve
            );
        } catch (Exception e) {
            throw new RuntimeException(
                "failed to instantiate genericified constructor: \n"
                + e
            );
        }
    }
    // prioritize the highest ranking and actively requesting socket; if
    // theres no requesting socket, prioritize the highest possesed socket.
    public SocketType[] getSortedSockets() {
        // fuck it
        // unchecked cast
        SocketType[] sockets = (SocketType[]) Array.newInstance(
            socketType,
            socketInstances.size()
        );
        socketInstances.values().toArray(sockets);

        acknowledgements.clear();

        return sockets;
    }

    public SocketType getSocket(SocketNamerator name) {
        return socketInstances.get(name);
    }

    public void requestAcknowledgement(SocketType socket) {
        acknowledgements.add(socket);
    }

    public boolean isAcknowledged(SocketType socket) {
        return !acknowledgements.contains(socket);
    }
}
