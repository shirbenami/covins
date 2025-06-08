#include <covins/comm_messages/MsgImage.hpp> // Ensure .hpp extension
#include <iostream> // For error/debug output

// Include OpenCV for image encoding/decoding and image processing functions like resize
#include <opencv2/imgcodecs.hpp> // For cv::imencode, cv::imdecode
#include <opencv2/imgproc.hpp>   // For cv::resize

namespace covins {

MsgImage::MsgImage()
    : timestamp(0.0)
{
    image = cv::Mat(); // Default initialize empty cv::Mat
}

std::unique_ptr<IMessage> MsgImage::clone() const {
    auto cloned_msg = std::make_unique<MsgImage>();
    cloned_msg->timestamp = this->timestamp;
    cloned_msg->image = this->image.clone(); // Deep copy for cv::Mat
    return cloned_msg;
}

void MsgImage::serialize(ISerializer& serializer) const {
    // IMPORTANT: Serialize the message type first for deserialization
    serializer.write("message_type", getType());

    serializer.write("timestamp", timestamp);

    // Serialize the image itself. Using JPEG compression for cv::Mat is common.
    if (!image.empty()) {
        std::vector<uint8_t> image_buffer;
        // Use JPEG compression for smaller size. Quality 80%.
        cv::imencode(".jpg", image, image_buffer, {cv::IMWRITE_JPEG_QUALITY, 80});
        serializer.write("image_data", image_buffer);
        serializer.write("image_rows", image.rows);
        serializer.write("image_cols", image.cols);
        serializer.write("image_type", image.type()); // Store type (e.g., CV_8UC1 for grayscale)
    } else {
        serializer.write("image_data", std::vector<uint8_t>{});
        serializer.write("image_rows", 0);
        serializer.write("image_cols", 0);
        serializer.write("image_type", 0);
    }
}

void MsgImage::deserialize(IDeserializer& deserializer) {
    // IMPORTANT: Deserialize the message type first
    std::string message_type_read = deserializer.readString("message_type");
    if (message_type_read != getType()) {
        std::cerr << "MsgImage: Deserialization type mismatch. Expected '" << getType() << "', got '" << message_type_read << "'" << std::endl;
        // Handle error, maybe throw exception or set invalid state
    }

    timestamp = deserializer.readDouble("timestamp");

    // Deserialize image
    std::vector<uint8_t> image_data = deserializer.readBinary("image_data");
    int image_rows = deserializer.readInt("image_rows");
    int image_cols = deserializer.readInt("image_cols");
    int image_type = deserializer.readInt("image_type");

    if (!image_data.empty() && image_rows > 0 && image_cols > 0) {
        // Decode image from buffer. Assuming it was encoded as JPEG.
        image = cv::imdecode(image_data, cv::IMREAD_UNCHANGED);
        if (image.empty()) {
            std::cerr << "MsgImage: Failed to imdecode image data." << std::endl;
        }
        // Basic check for dimensions/type consistency after decoding
        if (image.rows != image_rows || image.cols != image_cols || image.type() != image_type) {
             // Optional: perform conversion if types don't match, e.g., to grayscale or specific type
             // For instance, if original was CV_8UC1 but JPEG decoded to CV_8UC3:
             // cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
             cv::resize(image, image, cv::Size(image_cols, image_rows));
             image.convertTo(image, image_type);
        }

    } else {
        image = cv::Mat(); // Empty matrix
    }
}

void MsgImage::setImage(const cv::Mat& img) {
    image = img.clone(); // Ensure deep copy
}

const cv::Mat& MsgImage::getImage() const {
    return image;
}

} // namespace covins
