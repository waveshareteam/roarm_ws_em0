import { useEffect, useRef } from "react";

import Box from "@mui/material/Box";
import Paper from "@mui/material/Paper";
import IconButton from "@mui/material/IconButton";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";

const completeCheck = (pc: RTCPeerConnection) => {
  return new Promise<void>((resolve) => {
    if (pc.iceGatheringState === "complete") {
      resolve();
    } else {
      const checkState = () => {
        if (pc.iceGatheringState === "complete") {
          pc.removeEventListener("icegatheringstatechange", checkState);
          resolve();
        }
      };
      pc.addEventListener("icegatheringstatechange", checkState);
    }
  });
};
const offerRequest = (pc: RTCPeerConnection, topic: string, url: string) => {
  const offer = pc.localDescription;

  return fetch(url, {
    body: JSON.stringify({
      sdp: offer?.sdp,
      type: offer?.type,
      topic,
    }),
    headers: {
      "Content-Type": "application/json",
    },
    method: "POST",
  });
};

export type ImageViewProps = {
  offerUrl: string;
  topic?: string;
  config?: Record<string, unknown>;
};
export default function ImageView(props: ImageViewProps) {
  const { topic, config, offerUrl } = props;

  const videoRef = useRef<HTMLVideoElement>(null);
  const buttonRef = useRef<HTMLButtonElement>(null);

  useEffect(() => {
    if (topic === undefined || topic === "") {
      console.error("topic is undefined");
      return;
    }

    if (videoRef.current) {
      const videoEl = videoRef.current;
      const pc = new RTCPeerConnection(config);
      pc.addEventListener("track", (ev) => {
        if (ev.track.kind == "video") {
          videoEl.srcObject = ev.streams[0];
        }
      });

      const negotiate = async () => {
        try {
          pc.addTransceiver("video", { direction: "recvonly" });
          const offer = await pc.createOffer();
          await pc.setLocalDescription(offer);
          await completeCheck(pc);
          const response = await offerRequest(pc, topic, offerUrl);
          const answer = await response.json();
          await pc.setRemoteDescription(answer);
        } catch (error) {
          console.error(String(error));
        }
      };
      negotiate().then(() => {
        // console.log("negotiate done");
      });

      return () => {
        setTimeout(function () {
          pc.close();
        }, 500);
      };
    }
  }, [videoRef, config, topic, offerUrl]);

  const aspectRatio = "640 / 480";

  const startHandler = () => {
    if (videoRef.current) {
      videoRef.current.play();
    }
    if (buttonRef.current) {
      buttonRef.current.style.display = "none";
    }
  };

  return (
    <Paper
      elevation={3}
      sx={{
        width: "100%",
        height: "undefined",
        aspectRatio,
        bgcolor: "black",
        objectFit: "cover",
        borderRadius: 1,
        overflow: "hidden",
        // border: "1px solid #black",
      }}
    >
      <video
        ref={videoRef}
        // autoPlay={false}
        playsInline={true}
        width={"100%"}
        height={"100%"}
      ></video>

      <Box
        sx={{
          position: "absolute",
          top: 0,
          bottom: 0,
          left: 0,
          right: 0,
          display: "flex",
          alignItems: "center",
          justifyContent: "center",
        }}
      >
        <IconButton
          ref={buttonRef}
          onClick={startHandler}
          size="large"
          sx={{
            color: "white",
          }}
        >
          <PlayArrowIcon />
        </IconButton>
      </Box>
    </Paper>
  );
}
