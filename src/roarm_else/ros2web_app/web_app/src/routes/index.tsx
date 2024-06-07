import { BrowserRouter, Routes, Route } from "react-router-dom";
import App from "@/containers/App";

const AppRoutes = () => {
  return (
    <BrowserRouter>
      <Routes>
        {/*<Route path="/" element={<Home />} />*/}
        <Route path="/:appName" element={<App />} />
      </Routes>
    </BrowserRouter>
  );
};

export default AppRoutes;
