import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  async rewrites() {
    const api = process.env.FMS_API_ORIGIN ?? "http://127.0.0.1:8741";
    return [
      {
        source: "/fms-api/:path*",
        destination: `${api}/api/:path*`,
      },
    ];
  },
};

export default nextConfig;
